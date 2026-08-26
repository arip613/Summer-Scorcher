package frc.robot;


import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.framework.TimedRobot;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.Trigger;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.sim.RobotSimulation;
import frc.robot.AutoMovements.HeadingLock;
import frc.robot.AutoMovements.HeadingLockMath;
import frc.robot.AutoMovements.RightTriggerMath;
import frc.robot.AutoMovements.OutpostSetpoint;
import frc.robot.FlywheelSubsystem.DistanceCalc;
import frc.robot.FlywheelSubsystem.LookupTable;
import frc.robot.Intake.IntakePosition;
import frc.robot.Intake.intaker;
import frc.robot.FlywheelSubsystem.Drum;
import frc.robot.FlywheelSubsystem.Hood;
import frc.robot.FlywheelSubsystem.DrumStateMachine;
import frc.robot.FlywheelSubsystem.HoodStateMachine;
import frc.robot.IndexerSubsystem.Indexer;
import frc.robot.IndexerSubsystem.Hopper;
import frc.robot.autos.PointToPointAutos;
import frc.robot.util.ElasticLayoutUtil;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.vision.VisionSubsystem;
import frc.robot.AutoMovements.FieldPoints;
import frc.robot.fms.FmsSubsystem;
import frc.robot.currentPhase.phaseTimer;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.framework.RobotBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;



public class Robot extends TimedRobot {
  private static final boolean ENABLE_DASHBOARD = true;
  private static final String SIM_ROTATION_AXIS_KEY = "Simulation/Driver/RotationAxis";
  private static final String SIM_SHOOT_AXIS_KEY = "Simulation/Driver/ShootAxis";
  private static final int RAW_XBOX_BACK_BUTTON = 6;
  private Command autonomousCommand = Commands.none();
  private final Hardware hardware = new Hardware();

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final RobotSimulation simulation =
      RobotBase.isSimulation() ? new RobotSimulation(hardware, swerve) : null;
  private final ImuSubsystem imu = new ImuSubsystem(
      swerve.drivetrainPigeon,
      () -> swerve.getSimPose().getRotation().getDegrees(),
      () -> Math.toDegrees(swerve.getSimFieldVelocity().omega));

  private final VisionSubsystem vision = new VisionSubsystem(
      imu,
      () -> swerve.getDrivetrainState().Pose.getRotation().getDegrees(),
      hardware.leftLimelight, hardware.rightLimelight);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(imu, vision, swerve);
  private final HeadingLock headingLock = new HeadingLock(localization, swerve);
  private final DistanceCalc distanceCalc = new DistanceCalc(localization, headingLock);
  private final Drum drum = new Drum(
      hardware.drumA1,
      hardware.drumA2,
      hardware.drumA3,
      hardware.drumA4);
  private final Hood hood = new Hood(hardware.hoodMotor);
  private final LookupTable turretLookup = new LookupTable(distanceCalc, drum, hood);
  private final intaker intakeRoller = new intaker(
      hardware.intakeRollerMotorA,
      hardware.intakeRollerMotorB);
  private final IntakePosition intakePosition = new IntakePosition(hardware.intakePivotMotor);
  private final OutpostSetpoint outpost = new OutpostSetpoint(localization, swerve, intakePosition, intakeRoller);
  private final DrumStateMachine drumSM = new DrumStateMachine(drum);
  private final HoodStateMachine hoodSM = new HoodStateMachine(hood);
  private final Indexer indexer = new Indexer(hardware.indexerMotor, hardware.indexerMotor2);
  private final Hopper hopper = new Hopper(hardware.hopperMotor);



  private final FollowPath.Builder blinePathBuilder;
  private final phaseTimer phaseTimer = new phaseTimer();
  private final PointToPointAutos pointToPointAutos;
  private boolean prevOperatorX = false;
  private boolean prevOperatorB = false;
  private boolean prevOperatorA = false;
  private final Orchestra orchestra = new Orchestra();
  private final Field2d field2d = new Field2d();
  private final org.wpilib.math.controller.PIDController trenchYController =
      new org.wpilib.math.controller.PIDController(3.0, 0.0, 0.0);
  private Translation2d activePassTarget;
  private boolean rtShootMode = true;
  private static final double SHOOT_SPEED_THRESHOLD = 0.5; // m/s — don't feed if moving faster
  private phaseTimer.Phase lastPhase = null;
  private boolean warningRumbleSent = false;
  // Rumble pattern: array of {duration, pause, duration, pause, ...} in seconds
  // Negative values = rumble off (pause), positive = rumble on
  private double[] rumblePattern = null;
  private int rumblePatternIndex = 0;
  private double rumbleStepEndTime = 0;
  private int shootReadyFrames = 0;
  private static final int SHOOT_READY_FRAME_THRESHOLD = 2;

  
  public Robot() {
    // Moved off DriverStation in 2027; only the internal backend still exposes it.
    DriverStationBackend.silenceJoystickConnectionWarning(true);

    LifecycleSubsystemManager.ready();

    SmartDashboard.putData("Field", field2d);

    if (simulation != null) {
      simulation.initialize();
      SmartDashboard.putData("Simulation/DriverController", hardware.driverController.getHID());
      // Traditional Xbox/DirectInput mappings expose right-stick X on raw axis 4. This remains
      // dashboard-selectable for controllers that provide WPILib's logical Gamepad axis 2.
      SmartDashboard.putNumber(SIM_ROTATION_AXIS_KEY, 4);
      SmartDashboard.putNumber(SIM_SHOOT_AXIS_KEY, 3);
    }

  orchestra.addInstrument(hardware.drumA1);
  orchestra.addInstrument(hardware.drumA2);
  orchestra.addInstrument(hardware.drumA3);
  orchestra.addInstrument(hardware.drumA4);
    orchestra.addInstrument(hardware.hopperMotor);
    orchestra.addInstrument(hardware.hoodMotor);
  orchestra.addInstrument(hardware.indexerMotor);
  orchestra.addInstrument(hardware.indexerMotor2);
    orchestra.addInstrument(hardware.intakePivotMotor);
  orchestra.addInstrument(hardware.intakeRollerMotorA);
  orchestra.addInstrument(hardware.intakeRollerMotorB);
    orchestra.loadMusic("output.chrp");

    headingLock.setRedTargetPoint(FieldPoints.getHeadingLockRedPoint());
    headingLock.setBlueTargetPoint(FieldPoints.getHeadingLockBluePoint());


    registerNamedCommands();

    try {
      var ppConfig = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
        () -> localization.getPose(),                // Pose supplier
        (pose) -> localization.resetPose(pose),      // Pose reset
        () -> swerve.getRobotRelativeSpeeds(),       // Robot-relative speeds supplier
        (speeds, feedforwards) -> swerve.setRobotRelativeAutoSpeeds(speeds), // Drive robot-relative
        new PPHolonomicDriveController(
          new PIDConstants(5.0, 0.0, 0.0),           // Translation PID
          new PIDConstants(5.0, 0.0, 0.0)            // Rotation PID
        ),
        ppConfig,
        () -> FmsSubsystem.isRedAlliance(),          // Flip for red alliance
        swerve                                       // Drive subsystem requirement
      );
    } catch (Exception e) {
      DriverStationErrors.reportError("Failed to configure PathPlanner: " + e.getMessage(), e.getStackTrace());
    }

    // BLine-Lib global constraints (no GUI/JSON needed).
    Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
        4.5,   // max velocity m/s
        12.0,  // max acceleration m/s²
        540.0, // max rotational velocity deg/s
        860.0, // max rotational acceleration deg/s²
        0.03,  // end translation tolerance m
        2.0,   // end rotation tolerance deg
        0.2    // intermediate handoff radius m
    ));

    // AutoRoutine mirrors its red-source poses itself, so BLine must not flip them again.
    blinePathBuilder = new FollowPath.Builder(
        swerve,
        localization::getPose,
        swerve::getRobotRelativeSpeeds,
        swerve::setRobotRelativeAutoSpeeds,
        new PIDController(5.0, 0.0, 0.0),
        new PIDController(5.0, 0.0, 0.0),
        new PIDController(2.0, 0.0, 0.0));

    configureBindings();

    // Set up point-to-point auto chooser (shows on SmartDashboard as "Auto Chooser")
  pointToPointAutos = new PointToPointAutos(
    swerve, localization, blinePathBuilder, drum, drumSM, hoodSM,
    headingLock, turretLookup, indexer, hopper, intakeRoller, intakePosition);

    ElasticLayoutUtil.onBoot();

    // 2027 removed robotInit(); the constructor is the init hook now.
    SignalLogger.enableAutoLogging(false);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    field2d.setRobotPose(localization.getPose());
    FieldPoints.publishHeadingLockPoints();
    field2d.getObject("Red Hub").setPose(
        new org.wpilib.math.geometry.Pose2d(
            FieldPoints.getHeadingLockRedPoint(), Rotation2d.kZero));
    field2d.getObject("Blue Hub").setPose(
        new org.wpilib.math.geometry.Pose2d(
            FieldPoints.getHeadingLockBluePoint(), Rotation2d.kZero));
    

    // Publish shooter pose (robot-relative offset transformed to field coordinates)
    var shooterField = localization.getPose().transformBy(
        new Transform2d(FieldPoints.SHOOTER_POSE.getTranslation(), FieldPoints.SHOOTER_POSE.getRotation()));
    SmartDashboard.putNumberArray("Shooter/Pose",
        new double[]{shooterField.getX(), shooterField.getY(), shooterField.getRotation().getDegrees()});
    field2d.getObject("Shooter").setPose(shooterField);

  }

  @Override
  public void disabledInit() {
    ElasticLayoutUtil.onDisable();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Use the point-to-point auto chooser from SmartDashboard
    autonomousCommand = pointToPointAutos.getSelected();
    if (autonomousCommand == null) {
      autonomousCommand = Commands.none().withName("NoAutoSelected");
    }
    SmartDashboard.putString("Auto/Selected", autonomousCommand.getName());
    SmartDashboard.putBoolean("Auto/Running", true);
    CommandScheduler.getInstance().schedule(autonomousCommand);

    ElasticLayoutUtil.onEnable();
  }

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putBoolean(
        "Auto/Running", autonomousCommand != null && autonomousCommand.isScheduled());
  }

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    swerve.setFieldRelativeAutoSpeeds(new org.wpilib.math.kinematics.ChassisVelocities());
    SmartDashboard.putBoolean("Auto/Running", false);
  }

  @Override
  public void teleopInit() {
    // Cancel all auto commands but keep the current pose
    CommandScheduler.getInstance().cancelAll();
    // Stop all mechanisms
  drum.stop();
    hood.stopMotor();
    indexer.stop();
    hopper.stop();
    intakeRoller.stop();
    headingLock.disableLock();
    turretLookup.disable();
  drumSM.requestOff();
    hoodSM.requestOff();

    ElasticLayoutUtil.onEnable();

    phaseTimer.markTeleopStart();
    lastPhase = phaseTimer.getCurrentPhase();
    warningRumbleSent = false;
  }

  /** RumbleType no longer has a combined value, so drive both motors together. */
  private void setDriverBothRumble(double value) {
    var hid = hardware.driverController.getHID();
    hid.setRumble(RumbleType.LEFT_RUMBLE, value);
    hid.setRumble(RumbleType.RIGHT_RUMBLE, value);
  }

  @Override
  public void teleopPeriodic() {
    phaseTimer.Phase currentPhase = phaseTimer.getCurrentPhase();
    double remaining = phaseTimer.getSecondsRemainingInCurrentPhase();

    // 5 seconds before shift ends: three quick rumble pulses
    if (remaining <= 5.0 && remaining > 4.5 && !warningRumbleSent && rumblePattern == null) {
      // Pattern: on 0.15s, off 0.1s, on 0.15s, off 0.1s, on 0.15s
      rumblePattern = new double[]{0.15, 0.1, 0.15, 0.1, 0.15};
      rumblePatternIndex = 0;
      rumbleStepEndTime = 0;
      warningRumbleSent = true;
    }

    // Shift change: one long rumble
    if (lastPhase != null && currentPhase != lastPhase) {
      rumblePattern = new double[]{0.8};
      rumblePatternIndex = 0;
      rumbleStepEndTime = 0;
      warningRumbleSent = false;
    }
    lastPhase = currentPhase;

    // Drive the rumble pattern
    double now = org.wpilib.system.Timer.getTimestamp();
    if (rumblePattern != null) {
      if (rumbleStepEndTime == 0) {
        // Start current step
        boolean isOn = (rumblePatternIndex % 2) == 0;
        setDriverBothRumble(isOn ? 1.0 : 0.0);
        rumbleStepEndTime = now + rumblePattern[rumblePatternIndex];
      } else if (now >= rumbleStepEndTime) {
        rumblePatternIndex++;
        if (rumblePatternIndex >= rumblePattern.length) {
          // Pattern done
          setDriverBothRumble(0.0);
          rumblePattern = null;
        } else {
          boolean isOn = (rumblePatternIndex % 2) == 0;
          setDriverBothRumble(isOn ? 1.0 : 0.0);
          rumbleStepEndTime = now + rumblePattern[rumblePatternIndex];
        }
      }
    }

    // Operator X/B: adjust shooting angle override
    // NOTE: change behavior to set a fixed one-step override on rising edge so
    // a single X (or B) press moves the aim immediately regardless of prior presses.
    Gamepad opXbox = (Gamepad) hardware.operatorController.getHID();
    boolean xPressed = opXbox.getWestFaceButton();
    boolean bPressed = opXbox.getEastFaceButton();
    if (bPressed && !prevOperatorB) {
      // Move a single step to the right (absolute step from center)
      headingLock.setOperatorOverrideDeg(-1.5);
    }
    if (xPressed && !prevOperatorX) {
      // Move a single step to the left (absolute step from center)
      headingLock.setOperatorOverrideDeg(1.5);
    }
    boolean aPressed = opXbox.getSouthFaceButton();
    if (aPressed && !prevOperatorA) {
      headingLock.setOperatorOverrideDeg(0.0);
    }
    prevOperatorA = aPressed;
    prevOperatorB = bPressed;
    prevOperatorX = xPressed;

    // Phase telemetry
    if (ENABLE_DASHBOARD) {
      SmartDashboard.putString("Phase/Current", currentPhase.name());
      SmartDashboard.putNumber("Phase/ElapsedSec", phaseTimer.getElapsedSec());
      SmartDashboard.putNumber("Phase/SecsInPhase", phaseTimer.getSecondsIntoCurrentPhase());
      SmartDashboard.putNumber("Phase/SecsRemaining", phaseTimer.getSecondsRemainingInCurrentPhase());
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void utilityInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void utilityPeriodic() {}

  @Override
  public void utilityExit() {}

  @Override
  public void simulationPeriodic() {
    SmartDashboard.putNumber(
        "Simulation/Driver/LeftX", hardware.driverController.getLeftX());
    SmartDashboard.putNumber(
        "Simulation/Driver/LeftY", hardware.driverController.getLeftY());
    SmartDashboard.putNumber(
        "Simulation/Driver/RightX", hardware.driverController.getRightX());
    SmartDashboard.putNumber(
        "Simulation/Driver/RightY", hardware.driverController.getRightY());
    var driverHid = hardware.driverController.getHID();
    int axisCount = driverHid.getAxesMaximumIndex();
    double[] rawAxes = new double[axisCount];
    for (int axis = 0; axis < rawAxes.length; axis++) {
      rawAxes[axis] = driverHid.getRawAxis(axis);
    }
    SmartDashboard.putNumberArray("Simulation/Driver/RawAxes", rawAxes);
    SmartDashboard.putNumber("Simulation/Driver/RotationInput", getDriverRotationInput());
    SmartDashboard.putNumber("Simulation/Driver/ShootInput", getDriverShootInput());
    simulation.periodic();
  }

  private double getDriverRotationInput() {
    if (!RobotBase.isSimulation()) {
      return hardware.driverController.getRightX();
    }

    var driverHid = hardware.driverController.getHID();
    int axisCount = driverHid.getAxesMaximumIndex();
    if (axisCount <= 0) {
      return 0.0;
    }
    int requestedAxis = (int) Math.round(SmartDashboard.getNumber(SIM_ROTATION_AXIS_KEY, 4));
    if (requestedAxis < 0 || requestedAxis >= axisCount) {
      return 0.0;
    }
    return driverHid.getRawAxis(requestedAxis);
  }

  private double getDriverShootInput() {
    if (!RobotBase.isSimulation()) {
      return hardware.driverController.getRightTriggerAxis();
    }

    var driverHid = hardware.driverController.getHID();
    int axisCount = driverHid.getAxesMaximumIndex();
    int requestedAxis = (int) Math.round(SmartDashboard.getNumber(SIM_SHOOT_AXIS_KEY, 3));
    if (requestedAxis < 0 || requestedAxis >= axisCount) {
      return 0.0;
    }
    return driverHid.getRawAxis(requestedAxis);
  }

  private void updatePassingBehavior() {
    Pose2d robotPose = localization.getPose();
    // Reevaluate while held so driving to the other side updates the selected pass target without
    // requiring the driver to release and press the trigger again.
    activePassTarget = RightTriggerMath.closestPassTarget(
        robotPose.getTranslation(),
        FieldPoints.getAlliancePassTargetRight(),
        FieldPoints.getAlliancePassTargetLeft());

    Transform2d robotToShooter = new Transform2d(
        FieldPoints.SHOOTER_POSE.getTranslation(),
        FieldPoints.SHOOTER_POSE.getRotation());
    double passHeading = RightTriggerMath.targetHeadingDegrees(
        robotPose, robotToShooter, activePassTarget);
    double passDistance = RightTriggerMath.targetDistanceMeters(
        robotPose, robotToShooter, activePassTarget);
    double passRpm = LookupTable.getPassRpm(passDistance);

    swerve.snapsDriveRequest(passHeading);
    drumSM.requestRpm(passRpm);
    hoodSM.requestDegrees(LookupTable.PASS_HOOD_ANGLE_DEG);

    double headingError = HeadingLockMath.errorDegrees(
        passHeading, robotPose.getRotation().getDegrees());
    var speeds = swerve.getRobotRelativeSpeeds();
    double robotSpeed = Math.hypot(speeds.vx, speeds.vy);
    boolean headingGood = Math.abs(headingError) <= 5.0;
    boolean slowEnough = robotSpeed < SHOOT_SPEED_THRESHOLD;
    boolean passReady = headingGood && slowEnough && drum.isAtGoal() && hood.isAtGoal();

    field2d.getObject("Pass Target").setPose(
        new Pose2d(activePassTarget, Rotation2d.kZero));
    if (ENABLE_DASHBOARD) {
      SmartDashboard.putNumberArray(
          "Pass/TargetPose",
          new double[] {activePassTarget.getX(), activePassTarget.getY(), passHeading});
      SmartDashboard.putNumber("Pass/DistanceM", passDistance);
      SmartDashboard.putNumber("Pass/TargetHeadingDeg", passHeading);
      SmartDashboard.putNumber("Pass/HeadingError", headingError);
      SmartDashboard.putBoolean("Pass/HeadingGood", headingGood);
      SmartDashboard.putNumber("Pass/RPM", passRpm);
      SmartDashboard.putNumber("Pass/RobotSpeed", robotSpeed);
      SmartDashboard.putBoolean("Pass/SlowEnough", slowEnough);
      SmartDashboard.putBoolean("Pass/FlywheelReady", drum.isAtGoal());
      SmartDashboard.putBoolean("Pass/HoodReady", hood.isAtGoal());
      SmartDashboard.putBoolean("Pass/Ready", passReady);
    }

    if (passReady) {
      indexer.feed();
      hopper.feed();
    } else {
      indexer.stop();
      hopper.stop();
    }
  }

  private void clearPassingState() {
    activePassTarget = null;
    if (!ENABLE_DASHBOARD) {
      return;
    }
    SmartDashboard.putBoolean("Driver/PassingActive", false);
    SmartDashboard.putBoolean("Pass/HeadingGood", false);
    SmartDashboard.putBoolean("Pass/SlowEnough", false);
    SmartDashboard.putBoolean("Pass/FlywheelReady", false);
    SmartDashboard.putBoolean("Pass/HoodReady", false);
    SmartDashboard.putBoolean("Pass/Ready", false);
    SmartDashboard.putNumber("Pass/HeadingError", 0.0);
  }

  private void enterRightTriggerMode(boolean shooting) {
    rtShootMode = shooting;
    shootReadyFrames = 0;
    SmartDashboard.putBoolean("Driver/RT_ShootMode", shooting);

    if (shooting) {
      clearPassingState();
      turretLookup.enable();
      headingLock.enableForAlliance();
      intakeRoller.intake();
      hopper.feed();
      SmartDashboard.putBoolean("Driver/ShootingActive", true);
      SmartDashboard.putBoolean("Driver/PassingActive", false);
    } else {
      SmartDashboard.putBoolean("Driver/ShootingActive", false);
      turretLookup.disable();
      headingLock.disableLock();
      activePassTarget = RightTriggerMath.closestPassTarget(
          localization.getPose().getTranslation(),
          FieldPoints.getAlliancePassTargetRight(),
          FieldPoints.getAlliancePassTargetLeft());
      updatePassingBehavior();
      SmartDashboard.putBoolean("Driver/PassingActive", true);
    }
  }

  private void registerNamedCommands() {
    // Indexer states
    NamedCommands.registerCommand("IndexerOff", Commands.runOnce(() -> indexer.stop()));
    NamedCommands.registerCommand("IndexerIntake", Commands.runOnce(() -> indexer.intake()));
    NamedCommands.registerCommand("IndexerFeed", Commands.runOnce(() -> indexer.feed()));
    NamedCommands.registerCommand("IndexerReverse", Commands.runOnce(() -> indexer.reverse()));

    // Hopper states
    NamedCommands.registerCommand("HopperOff", Commands.runOnce(() -> hopper.stop()));
    NamedCommands.registerCommand("HopperIntake", Commands.runOnce(() -> hopper.intake()));
    NamedCommands.registerCommand("HopperFeed", Commands.runOnce(() -> hopper.feed()));
    NamedCommands.registerCommand("HopperReverse", Commands.runOnce(() -> hopper.reverse()));

    // Intaker (roller) states
    NamedCommands.registerCommand("IntakerOff", Commands.runOnce(() -> intakeRoller.stop()));
    NamedCommands.registerCommand("IntakerIntake", Commands.runOnce(() -> intakeRoller.intake()));
    NamedCommands.registerCommand("IntakerFeed", Commands.runOnce(() -> intakeRoller.feed()));
    NamedCommands.registerCommand("IntakerReverse", Commands.runOnce(() -> intakeRoller.reverse()));

    // Intake position states
    NamedCommands.registerCommand("IntakePositionDeploy", Commands.runOnce(() -> intakePosition.deploy()));
    NamedCommands.registerCommand("IntakePositionRetract", Commands.runOnce(() -> intakePosition.retract()));

  // Drum states
  NamedCommands.registerCommand("ShooterOff", Commands.runOnce(() -> drumSM.requestOff()));
  NamedCommands.registerCommand("ShooterSpin", Commands.runOnce(() -> drumSM.requestRpm(3200.0)));

    // Hood states
    NamedCommands.registerCommand("HoodOff", Commands.runOnce(() -> hoodSM.requestOff()));

    // Heading lock + lookup table: face target and spin up
    NamedCommands.registerCommand("FaceTarget", Commands.runOnce(() -> {
      headingLock.enableForAlliance();
      turretLookup.enable();
    }));
    NamedCommands.registerCommand("FaceTargetOff", Commands.runOnce(() -> {
      headingLock.disableLock();
      turretLookup.disable();
  drumSM.requestOff();
      hoodSM.requestOff();
    }));
  }

  private void configureBindings() {
    // The controller is exposed in the traditional zero-based raw Xbox layout. Its physical
    // View/Back button is raw button 6; CommandGamepad.back() reads logical Gamepad button 4.
    hardware.driverController.button(RAW_XBOX_BACK_BUTTON).onTrue(
        Commands.runOnce(() -> {
          double heading = FmsSubsystem.isRedAlliance() ? 180.0 : 0.0;
          localization.resetGyro(Rotation2d.fromDegrees(heading));
        }).ignoringDisable(true));


    hardware.operatorController.northFace().whileTrue(
      org.wpilib.command2.Commands.startEnd(
        () -> {
          drum.dutyCycle(1);
  
        },
        () -> {
         drum.dutyCycle(0);
    
        }
      )
    );
  
      hardware.operatorController.leftTrigger(0.1).whileTrue(
      org.wpilib.command2.Commands.startEnd(
        () -> {
          drum.dutyCycle(0.5);
  
        },
        () -> {
          drum.stop();
    
        }
      )
    );

    

    hardware.driverController.rightBumper().whileTrue(
      org.wpilib.command2.Commands.startEnd(
        () -> {
          intakePosition.pulse();
  
        },
        () -> {
intakePosition.deploy();    
        }
      )
    );
    
    hardware.driverController.leftBumper().whileTrue(
      org.wpilib.command2.Commands.startEnd(
        () -> {
          intakeRoller.reverse();
  
        },
        () -> {
         intakeRoller.stop();
    
        }
      )
    );

    hardware.driverController.dpadUp().whileTrue(
      org.wpilib.command2.Commands.startEnd(
        () -> {
          intakePosition.retract();
  
        },
        () -> {
          intakePosition.deploy();
        }
      )
    );


    

    hardware.driverController.leftTrigger(0.1).whileTrue(
      org.wpilib.command2.Commands.startEnd(
        () -> {
          
        intakePosition.deploy();
        intakeRoller.intake();

  
        },
        () -> {
              

        intakeRoller.stop();
        hopper.stop();
    
        }
      )
    );


   hardware.driverController.westFace().whileTrue(
      outpost.travelToOutpost()
    );


    hardware.driverController.northFace().whileTrue(
      org.wpilib.command2.Commands.startEnd(
        () -> {
          drum.spinDrum(2100);
          hood.setAngleDegrees(-40);
          hopper.feed();
          indexer.feed();
        },
        () -> {
          drum.stop();
          hood.setAngleDegrees(0);
          hopper.stop();
          indexer.stop();
        }
      )
    );



  
    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  if (RobotState.isTeleop()) {
                    swerve.driveTeleop(
                        hardware.driverController.getLeftX(),
                        hardware.driverController.getLeftY(),
                        getDriverRotationInput());
                  }
                })
            .withName("DefaultSwerveCommand"));




    Trigger shootTrigger = RobotBase.isSimulation()
        ? new Trigger(() -> getDriverShootInput() > 0.1)
        : hardware.driverController.rightTrigger(0.1);
    shootTrigger.whileTrue(
      org.wpilib.command2.Commands.startEnd(
        () -> {
          double robotX = localization.getPose().getX();
          enterRightTriggerMode(FieldPoints.isInShootZone(robotX));
        },
        () -> {
          boolean wasShootMode = rtShootMode;

          if (wasShootMode) {
            turretLookup.disable();
            headingLock.disableLock();
            drumSM.requestOff();
            hoodSM.requestOff();
            hood.setAngleDegrees(0);
            intakePosition.deploy();
            indexer.stop();
            intakeRoller.stop();
            hopper.stop();
            if (ENABLE_DASHBOARD) SmartDashboard.putBoolean("Driver/ShootingActive", false);
          } else {
            swerve.normalDriveRequest();
            drumSM.requestOff();
            hoodSM.requestOff();
            hood.setAngleDegrees(0);
            indexer.stop();
            intakeRoller.stop();
            hopper.stop();
            intakePosition.deploy();
            clearPassingState();
            if (ENABLE_DASHBOARD) {
              SmartDashboard.putBoolean("Driver/PassingActive", false);
            }
          }
        }
      ).alongWith(
        org.wpilib.command2.Commands.run(() -> {
          boolean desiredShootMode = FieldPoints.isInShootZone(localization.getPose().getX());
          if (desiredShootMode != rtShootMode) {
            enterRightTriggerMode(desiredShootMode);
          }
          boolean shootMode = rtShootMode;

          if (shootMode) {
            var params = turretLookup.getParameters();
            double rpm = params.flywheelRpm();
            double hoodRad = params.hoodAngleRad();
            drumSM.requestRpm(rpm);
            hoodSM.requestDegrees(Math.toDegrees(hoodRad));
            var speeds = swerve.getRobotRelativeSpeeds();
            double robotSpeed = Math.hypot(speeds.vx, speeds.vy);
            boolean slowEnough = robotSpeed < SHOOT_SPEED_THRESHOLD;
            boolean allReady = params.isValid() && slowEnough && drum.isAtGoal() && headingLock.isSettled();
            if (allReady) {
              shootReadyFrames++;
            } else {
              shootReadyFrames = 0;
            }
            if (ENABLE_DASHBOARD) {
              SmartDashboard.putNumber("Driver/RobotSpeed", robotSpeed);
              SmartDashboard.putBoolean("Driver/SlowEnoughToShoot", slowEnough);
              SmartDashboard.putNumber("Driver/ShootReadyFrames", shootReadyFrames);
            }
            if (shootReadyFrames >= SHOOT_READY_FRAME_THRESHOLD) {
              indexer.feed();
              hopper.feed();
            } else {
              indexer.stop();
              hopper.stop();
            }
          } else {
            updatePassingBehavior();
          }
        })
      ).alongWith(
        org.wpilib.command2.Commands.sequence(
          org.wpilib.command2.Commands.waitSeconds(1.75),
          org.wpilib.command2.Commands.runOnce(() -> intakePosition.shooter())
        )
      )
    );


  
  }
    
}
