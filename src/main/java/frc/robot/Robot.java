package frc.robot;


import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.framework.TimedRobot;
import org.wpilib.system.DataLogManager;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;
import frc.robot.imu.BumpCrossingTracker;
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
import frc.robot.Intake.BeamBreak;
import frc.robot.Intake.IntakePosition;
import frc.robot.Intake.intaker;
import frc.robot.FlywheelSubsystem.Drum;
import frc.robot.FlywheelSubsystem.Hood;
import frc.robot.FlywheelSubsystem.DrumStateMachine;
import frc.robot.FlywheelSubsystem.DrumTuner;
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

  // Legacy XboxController axis layout, which is what the old Driver Station reports.
  // WPILib 2027's Gamepad class assumes a different order -- it puts RightX/RightY on 2/3 and the
  // triggers on 4/5 -- so getRightX() and rightTrigger() read the wrong physical control here.
  // Read the raw indices directly instead of going through the named accessors.
  //   0 LeftX   1 LeftY   2 LeftTrigger   3 RightTrigger   4 RightX   5 RightY
  private static final int LEGACY_AXIS_LEFT_TRIGGER = 2;
  private static final int LEGACY_AXIS_RIGHT_TRIGGER = 3;
  private static final int LEGACY_AXIS_RIGHT_X = 4;
  private static final double TRIGGER_PRESS_THRESHOLD = 0.1;

  /**
   * Starts on-robot logging to a WPILOG file.
   *
   * <p>DogLog writes through WPILib's DataLogManager, whose makeLogDir() prefers a mounted USB
   * drive at /u when one is writable and falls back to <operating dir>/logs otherwise. So plugging
   * in a flash drive is all that is needed to log to it -- no path is hardcoded here, and the robot
   * still logs to internal storage when no drive is present.
   *
   * <p>Files land in /u/logs and open in AdvantageScope. The analyze-wpilog skill under
   * .claude/skills covers reading them programmatically.
   */
  private void configureLogging() {
    // Not started in simulation: it would drop wpilog files into the project directory on every
    // run of the test suite.
    if (RobotBase.isSimulation()) {
      return;
    }

    DataLogManager.start();

    DogLog.setOptions(
        new DogLogOptions()
            // Driver Station state (enabled/autonomous/test/estop) and joystick data. The
            // analyze-wpilog skill relies on the /DS: entries these produce.
            .withCaptureDs(true)
            // System stats: CAN utilization, battery voltage, loop timing.
            .withLogExtras(true)
            // Mirror System.out into the log so console errors line up with robot state.
            .withCaptureConsole(true)
            // Tunables publish to NetworkTables only when not in a real match.
            .withNtTunables(() -> !RobotState.isFMSAttached()));

    DogLog.log("Robot/LogDir", DataLogManager.getLogDir());
  }

  /** Raw axis read that tolerates a controller reporting fewer axes than expected. */
  private double rawAxis(CommandGamepad controller, int axis) {
    var hid = controller.getHID();
    if (axis < 0 || axis >= hid.getAxesMaximumIndex()) {
      return 0.0;
    }
    return hid.getRawAxis(axis);
  }
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
  @SuppressWarnings("unused")
  private final BeamBreak beamBreak = new BeamBreak(hardware.beamBreakSensor);
  // Kept constructed but unbound -- the X button that drove to the outpost is now a forceful stow.
  @SuppressWarnings("unused")
  private final OutpostSetpoint outpost = new OutpostSetpoint(localization, swerve, intakePosition, intakeRoller);
  private final DrumStateMachine drumSM = new DrumStateMachine(drum);
  @SuppressWarnings("unused")
  private final DrumTuner drumTuner = new DrumTuner(drum, drumSM);
  private final HoodStateMachine hoodSM = new HoodStateMachine(hood);
  private final Indexer indexer = new Indexer(hardware.indexerMotor, hardware.indexerMotor2);
  private final Hopper hopper = new Hopper(hardware.hopperMotor);



  private final FollowPath.Builder blinePathBuilder;
  private final BumpCrossingTracker bumpCrossingTracker;
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
  /**
   * Holds the shot gate open through brief gate dropouts so the feed does not stutter.
   *
   * <p>0.08s. Measured release velocity over a volley: target 2800, mean 2711, min 2615. With
   * RPM_TOLERANCE at 80 no ball should leave below 2720, so a 0.25s window was holding the gate
   * open ~105 RPM past the limit and letting slow balls out -- that was the shot spread. Keep this
   * short enough that the gate closes during a dip, but long enough to bridge a single-frame
   * heading flicker.
   */
  private final Debouncer shootReadyDebouncer =
      new Debouncer(0.08, Debouncer.DebounceType.kFalling);
  /** Frames each shoot gate blocked on, reset when the trigger is pressed. */
  private int blockedInvalid = 0;
  private int blockedTooFast = 0;
  private int blockedDrum = 0;
  private int blockedHeading = 0;

  
  public Robot() {
    // Moved off DriverStation in 2027; only the internal backend still exposes it.
    DriverStationBackend.silenceJoystickConnectionWarning(true);

    configureLogging();

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
    //
    // These MUST satisfy P * maxVelocity <= maxAcceleration for each axis, using the gains
    // passed to FollowPath.Builder below. BLine drives with a pure P controller behind a
    // velocity clamp and a rate limiter (FollowPath.execute phase 6). A P law commands
    // v = P * error, so it demands a deceleration of P * v -- peaking at P * maxVelocity right
    // as it comes out of the velocity clamp. If the rate limiter cannot supply that, the robot
    // arrives at the target still faster than the P law wants, sails past it, and oscillates
    // back. Rotation was the worst offender: 5.0 * 540 deg/s demanded 2700 deg/s^2 against an
    // 860 deg/s^2 limit, so every turn larger than ~108 deg overshot by construction.
    Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
        4.5,   // max velocity m/s          (2.5 * 4.5 = 11.25 <= 12.0)
        12.0,  // max acceleration m/s²
        200.0, // max rotational velocity deg/s   (4.0 * 200 = 800 <= 860)
        860.0, // max rotational acceleration deg/s²
        0.03,  // end translation tolerance m
        2.0,   // end rotation tolerance deg
        0.2    // intermediate handoff radius m
    ));

    bumpCrossingTracker = new BumpCrossingTracker(imu, localization::resetTranslationOnly);

    // AutoRoutine mirrors its red-source poses itself, so BLine must not flip them again.
    // The speeds consumer is wrapped rather than the follower because BLine has no follower
    // seam: this is the same interception point 581's BumpCrossingFollower uses, one layer down.
    blinePathBuilder = new FollowPath.Builder(
        swerve,
        localization::getPose,
        swerve::getRobotRelativeSpeeds,
        speeds ->
            swerve.setRobotRelativeAutoSpeeds(
                bumpCrossingTracker.applyCrossingOverride(
                    speeds, localization.getPose().getRotation())),
        new PIDController(2.5, 0.0, 0.0),  // translation: bounded by 12.0 / 4.5 = 2.66
        new PIDController(4.0, 0.0, 0.0),  // rotation:    bounded by 860 / 200 = 4.3
        new PIDController(2.0, 0.0, 0.0)); // cross-track: unclamped by BLine, left alone

    configureBindings();

    // Set up point-to-point auto chooser (shows on SmartDashboard as "Auto Chooser")
  pointToPointAutos = new PointToPointAutos(
    swerve, localization, blinePathBuilder, bumpCrossingTracker, drum, drumSM, hoodSM,
    headingLock, turretLookup, indexer, hopper, intakeRoller, intakePosition, beamBreak);

    ElasticLayoutUtil.onBoot();

    // 2027 removed robotInit(); the constructor is the init hook now.
    SignalLogger.enableAutoLogging(false);
  }

  /**
   * Publishes every raw driver axis alongside the named accessors. WPILib 2027's Gamepad puts
   * RightX/RightY at raw axes 2/3 and the triggers at 4/5, where the legacy XboxController layout
   * had them at 4/5 and 2/3 -- the same remap that put the physical Back button at raw 6. This
   * shows which physical stick each named accessor is actually reading.
   */
  private void publishDriverAxisDiagnostics() {
    var driverHid = hardware.driverController.getHID();
    int axisCount = driverHid.getAxesMaximumIndex();
    double[] rawAxes = new double[axisCount];
    for (int axis = 0; axis < rawAxes.length; axis++) {
      rawAxes[axis] = driverHid.getRawAxis(axis);
    }
    SmartDashboard.putNumberArray("Driver/RawAxes", rawAxes);
    SmartDashboard.putNumberArray(
        "Driver/Named",
        new double[] {
          hardware.driverController.getLeftX(),
          hardware.driverController.getLeftY(),
          hardware.driverController.getRightX(),
          hardware.driverController.getRightY()
        });
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    publishDriverAxisDiagnostics();
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
    // The intake's initial state is OFF, which commands CoastOut -- the arm is unheld and settles
    // wherever gravity leaves it. Hold it at the deployed position instead, so enable puts the arm
    // at the tuned working height (Intake/Tune/DeployRotations) rather than letting it sag.
    // This said retract(), against the stated intent right above it, so the arm lifted on every
    // teleop enable.
    intakePosition.deploy();

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
      return rawAxis(hardware.driverController, LEGACY_AXIS_RIGHT_X);
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
      return rawAxis(hardware.driverController, LEGACY_AXIS_RIGHT_TRIGGER);
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
    blockedInvalid = 0;
    blockedTooFast = 0;
    blockedDrum = 0;
    blockedHeading = 0;
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
  
      new Trigger(() ->
          rawAxis(hardware.operatorController, LEGACY_AXIS_LEFT_TRIGGER) > TRIGGER_PRESS_THRESHOLD
      ).whileTrue(
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


    

    new Trigger(() ->
        rawAxis(hardware.driverController, LEGACY_AXIS_LEFT_TRIGGER) > TRIGGER_PRESS_THRESHOLD
    ).whileTrue(
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


    // Forceful stow: pulls the arm all the way in with no motion profile.
    hardware.driverController.westFace().onTrue(
      org.wpilib.command2.Commands.runOnce(() -> intakePosition.forceStow())
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




    Trigger shootTrigger = new Trigger(() -> getDriverShootInput() > TRIGGER_PRESS_THRESHOLD);
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
            // Must come from the subsystem, not re-derived from getParameters(). LookupTable is
            // already commanding the drum and hood every loop, and re-deriving here ignored the
            // manual tuning override -- the two then alternated 0 and -15 degrees at 50 Hz.
            double rpm = turretLookup.getActiveRpm();
            double hoodDeg = turretLookup.getActiveHoodDeg();
            drumSM.requestRpm(rpm);
            hoodSM.requestDegrees(hoodDeg);
            var speeds = swerve.getRobotRelativeSpeeds();
            double robotSpeed = Math.hypot(speeds.vx, speeds.vy);
            boolean slowEnough = robotSpeed < SHOOT_SPEED_THRESHOLD;
            boolean validNow = params.isValid();
            boolean drumReady = drum.isAtGoal();
            boolean headingReady = headingLock.isSettled();
            // Heading is NOT a gate. 581, 2910 and 6328 all gate the shot on flywheel (and hood)
            // readiness only, and let the drivetrain converge on heading in parallel -- none of
            // them block feeding on it. Gating on it here stalled the shot indefinitely whenever
            // the snap settled a couple of degrees short, which it does consistently. The heading
            // lock still runs and still aims; it just no longer holds the trigger hostage.
            boolean allReadyRaw = validNow && slowEnough && drumReady;
            // Falling debounce: a single bad frame used to zero shootReadyFrames, which stopped the
            // indexer and hopper mid-shot. Heading settles in and out across the 2 degree tolerance
            // while the swerve holds angle, so the feed stuttered instead of running continuously
            // and balls never got a clean run at the shooter. Rising edge is still immediate.
            boolean allReady = shootReadyDebouncer.calculate(allReadyRaw);
            if (allReady) {
              shootReadyFrames++;
            } else {
              shootReadyFrames = 0;
            }

            // Any one of these dropping for a single frame resets shootReadyFrames and stops the
            // feed, which reads as "it just doesn't shoot sometimes". Counting blocked frames per
            // gate shows which one is actually responsible over a whole trigger hold.
            String blocker;
            if (!validNow) {
              blocker = String.format("shot params invalid (%.2f m)", params.distance());
              blockedInvalid++;
            } else if (!slowEnough) {
              blocker = String.format("moving %.2f m/s", robotSpeed);
              blockedTooFast++;
            } else if (!drumReady) {
              blocker = String.format(
                  "drum off RPM by %.0f", turretLookup.getActiveRpm() - drum.getRpm());
              blockedDrum++;
            } else {
              blocker = "ready";
            }
            // Heading no longer blocks, but keep counting frames fired while off-target so aim
            // quality stays visible instead of silently degrading.
            if (!headingReady) {
              blockedHeading++;
            }

            if (ENABLE_DASHBOARD) {
              SmartDashboard.putNumber("Driver/RobotSpeed", robotSpeed);
              SmartDashboard.putBoolean("Driver/SlowEnoughToShoot", slowEnough);
              SmartDashboard.putNumber("Driver/ShootReadyFrames", shootReadyFrames);
              SmartDashboard.putString("Driver/BlockedBy", blocker);
              SmartDashboard.putNumber("Driver/BlockedFrames/Invalid", blockedInvalid);
              SmartDashboard.putNumber("Driver/BlockedFrames/TooFast", blockedTooFast);
              SmartDashboard.putNumber("Driver/BlockedFrames/DrumRPM", blockedDrum);
              // Not a blocker any more: frames fed while the heading was outside tolerance.
              SmartDashboard.putNumber("Driver/FramesFedOffHeading", blockedHeading);
              SmartDashboard.putBoolean("Driver/HeadingSettled", headingReady);
            }
            if (shootReadyFrames >= SHOOT_READY_FRAME_THRESHOLD) {
              indexer.feed();
              hopper.feed();
              // Feeding: sweep the arm end to end to keep balls moving toward the indexer.
              intakePosition.swing();
            } else {
              indexer.stop();
              hopper.stop();
              // Waiting on the shot gate with nothing at the sensor -- run the staged agitation
              // (20% of travel, then 50%, then all the way down) to work a ball into position.
              // With a ball already there, leave the arm down and let the gate do its job.
              if (!beamBreak.hasBall()) {
                intakePosition.pulse();
              } else {
                intakePosition.deploy();
              }
            }
          } else {
            updatePassingBehavior();
          }
        })
      ).alongWith(
        org.wpilib.command2.Commands.sequence(
          org.wpilib.command2.Commands.waitSeconds(1.75),
          org.wpilib.command2.Commands.runOnce(() -> intakePosition.deploy())
        )
      )
    );


  
  }
    
}
