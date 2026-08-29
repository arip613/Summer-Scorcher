package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import frc.robot.fms.FmsSubsystem;
import org.wpilib.math.util.MathUtil;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.generated.CompBotTunerConstants;
import frc.robot.generated.CompBotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.MathHelpers;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.util.Map;

public class SwerveSubsystem extends StateMachine<SwerveState> {
  public static final double MaxSpeed = 4.75;
  private static final double maxAngularRate = Units.rotationsToRadians(4);
  // Must be a plain double, not a Rotation2d. In 2027 Rotation2d stores only cos/sin and
  // getRadians() is atan2(sin, cos), so it wraps to [-pi, pi] -- Rotation2d.fromRotations(2)
  // .getRadians() returned 4*pi in 2026 but returns 0 here, silently killing all rotation.
  private static final double TELEOP_MAX_ANGULAR_RATE = Units.rotationsToRadians(2);

  private static final double LEFT_X_DEADBAND = 0.05;
  private static final double LEFT_Y_DEADBAND = 0.05;
  private static final double RIGHT_X_DEADBAND = 0.15;

  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
  private static final double SIM_MAX_TRANSLATIONAL_ACCEL = 10.0;
  private static final double SIM_MAX_ANGULAR_ACCEL = 30.0;

  private static final PhoenixPIDController ORIGINAL_HEADING_PID =
      RobotConfig.get().swerve().snapController();
  private static final double HEADING_MIN_COMMAND = 0.05;

  private static final InterpolatingDoubleTreeMap ELEVATOR_HEIGHT_TO_SLOW_MODE =
      InterpolatingDoubleTreeMap.ofEntries(Map.entry(0.0, 1.0));

  public final TunerSwerveDrivetrain drivetrain =
    new TunerSwerveDrivetrain(
      CompBotTunerConstants.DrivetrainConstants,
      CompBotTunerConstants.FrontLeft,
      CompBotTunerConstants.FrontRight,
      CompBotTunerConstants.BackLeft,
      CompBotTunerConstants.BackRight);

  public final Pigeon2 drivetrainPigeon = drivetrain.getPigeon2();

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.015)
          .withRotationalDeadband(maxAngularRate * 0.015);

  private final SwerveRequest.FieldCentricFacingAngle driveToAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDeadband(MaxSpeed * 0.01)
          .withMaxAbsRotationalRate(maxAngularRate);

  private double lastSimTime;
  private Notifier simNotifier = null;
  private final Object simPoseLock = new Object();
  private Pose2d simPose = Pose2d.kZero;
  private ChassisVelocities simFieldVelocity = new ChassisVelocities();

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private ChassisVelocities robotRelativeSpeeds = new ChassisVelocities();
  private ChassisVelocities fieldRelativeSpeeds = new ChassisVelocities();
  private double goalSnapAngle = 0;

  private ChassisVelocities teleopSpeeds = new ChassisVelocities();

  private ChassisVelocities autoSpeeds = new ChassisVelocities();

  
  private final Timer timeSinceAutoSpeeds = new Timer();
  private double teleopSlowModePercent = 0.0;
  private boolean vyOverrideActive = false;
  private double rawControllerXValue = 0.0;
  private double rawControllerYValue = 0.0;

  public ChassisVelocities getRobotRelativeSpeeds() {
    return robotRelativeSpeeds;
  }

  public ChassisVelocities getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
  }

  public SwerveDriveState getDrivetrainState() {
    return drivetrainState;
  }

  public double[] getDriveStatorCurrents() {
    var modules = drivetrain.getModules();
    double[] currents = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      try {
        currents[i] = modules[i].getDriveMotor().getStatorCurrent().getValueAsDouble();
      } catch (Exception ex) {
        currents[i] = 0.0;
      }
    }
    return currents;
  }

  public double getDriveStatorCurrentAvg() {
    double[] currents = getDriveStatorCurrents();
    if (currents.length == 0) {
      return 0.0;
    }
    double sum = 0.0;
    for (double current : currents) {
      sum += current;
    }
    return sum / currents.length;
  }

  public double getDriveStatorCurrentMax() {
    double[] currents = getDriveStatorCurrents();
    double max = 0.0;
    for (double current : currents) {
      max = Math.max(max, current);
    }
    return max;
  }

  /**
   * Maximum rate the snap target itself may move, in degrees per second.
   *
   * <p>Not a limit on how fast the robot turns -- that is the request's own max rotational rate.
   * This bounds how fast the *goal* is allowed to move, which is a different thing: the aim heading
   * is recomputed every loop from the pose estimate, so a pose jump moves the target instantly and
   * the snap faithfully chases it.
   *
   * <p>In match AZGLE4_Q29 the estimate tore by 2-3m while aiming from ~1.5m out, which inverts the
   * bearing -- Pass/DistanceM went 3.17m to 1.67m in a single 20ms frame -- and the robot whipped
   * about 220 degrees in under a second. 540 deg/s lets a genuine aim change track a moving robot
   * comfortably while making a single-frame pose jump a bounded nudge instead of a spin.
   */
  private static final double MAX_SNAP_TARGET_SLEW_DEG_PER_SEC = 540.0;

  private double lastSnapUpdateTimestamp = -1.0;

  public void setSnapToAngle(double angle) {
    double now = Timer.getTimestamp();
    double dt = lastSnapUpdateTimestamp < 0 ? 0.0 : now - lastSnapUpdateTimestamp;
    lastSnapUpdateTimestamp = now;

    if (dt <= 0.0 || dt > 0.25) {
      // First call, or a gap long enough that the previous target is meaningless -- accept as-is
      // rather than slewing across a stale value.
      goalSnapAngle = angle;
    } else {
      // Shortest-path error, so wrapping through +/-180 costs nothing.
      double error = MathUtil.inputModulus(angle - goalSnapAngle, -180.0, 180.0);
      double maxStep = MAX_SNAP_TARGET_SLEW_DEG_PER_SEC * dt;
      goalSnapAngle =
          MathUtil.inputModulus(
              goalSnapAngle + Math.clamp(error, -maxStep, maxStep), -180.0, 180.0);
    }

    SmartDashboard.putNumber("Swerve/SnapTargetRequested", angle);
    SmartDashboard.putNumber("Swerve/SnapTargetUsed", goalSnapAngle);

    if (RobotState.isAutonomous()) {
      sendSwerveRequest();
    }
  }

  private double elevatorHeight;

  public SwerveSubsystem() {
    super(SubsystemPriority.SWERVE, SwerveState.TELEOP);

    if (Utils.isSimulation()) {
      // Phoenix 26.50.0-alpha-1's desktop odometry thread continuously waits on status frames
      // that never arrive in the 2027 alpha stack. Our simulation pose below replaces it.
      drivetrain.getOdometryThread().stop();
      startSimThread();
    }

    driveToAngle.HeadingController = new PhoenixPIDController(
        ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD()) {
      @Override
      public double calculate(double measurement, double setpoint, double currentTimestamp) {
        double output = super.calculate(measurement, setpoint, currentTimestamp);
        if (!atSetpoint() && Math.abs(output) < HEADING_MIN_COMMAND) {
          output = Math.copySign(HEADING_MIN_COMMAND, output);
        }
        return output;
      }
    };
    driveToAngle.HeadingController.setIZone(ORIGINAL_HEADING_PID.getIZone());
    driveToAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveToAngle.HeadingController.setTolerance(0.01);

    // Use CTRE's estimator baseline instead of treating wheel/gyro odometry as nearly perfect.
    // This lets accepted global measurements correct ordinary wheel slip through normal fusion.
    drivetrain.setStateStdDevs(new Matrix<>(VecBuilder.fill(0.1, 0.1, 0.1)));
    timeSinceAutoSpeeds.start();
  }

  public void setFieldRelativeAutoSpeeds(ChassisVelocities speeds) {
    autoSpeeds = speeds;
    timeSinceAutoSpeeds.reset();
    sendSwerveRequest();
  }

  public void setRobotRelativeAutoSpeeds(ChassisVelocities speeds) {
    setFieldRelativeAutoSpeeds(
        speeds.toFieldRelative(drivetrainState.Pose.getRotation()));
  }

 

  @Override
  protected SwerveState getNextState(SwerveState currentState) {
    return switch (currentState) {
      case AUTO, TELEOP -> RobotState.isAutonomous() ? SwerveState.AUTO : SwerveState.TELEOP;
      case AUTO_SNAPS, TELEOP_SNAPS ->
          RobotState.isAutonomous() ? SwerveState.AUTO_SNAPS : SwerveState.TELEOP_SNAPS;
    };
  }

  public void driveTeleop(double x, double y, double theta) {
    rawControllerXValue = x;
    rawControllerYValue = y;
    double leftY =
        -1.0
            * MathHelpers.signedExp(
                ControllerHelpers.deadbandJoystickValue(y, LEFT_Y_DEADBAND), 2.0);
    double leftX =
        MathHelpers.signedExp(ControllerHelpers.deadbandJoystickValue(x, LEFT_X_DEADBAND), 2.0);
    double rightX =
        MathHelpers.signedExp(
            ControllerHelpers.deadbandJoystickValue(theta, RIGHT_X_DEADBAND), 1.3);

    if (RobotConfig.get().swerve().invertRotation()) {
      rightX *= -1.0;
    }

    if (RobotConfig.get().swerve().invertX()) {
      leftX *= -1.0;
    }

    if (RobotConfig.get().swerve().invertY()) {
      leftY *= -1.0;
    }

    // Must be the same alliance source the rest of the robot uses. This previously read
    // MatchState.getAlliance().orElse(BLUE) while FmsSubsystem defaults to RED on a real robot, so
    // with no alliance selected on the DS the drive refused to flip while HeadingLock, the autos
    // and FieldPoints all treated the robot as red -- translation backwards, rotation fine.
    if (FmsSubsystem.isRedAlliance()) {
      leftX *= -1.0;
      leftY *= -1.0;
    }

    Translation2d mappedpose = ControllerHelpers.fromCircularDiscCoordinates(leftX, leftY);
    double mappedX = mappedpose.getX();
    double mappedY = mappedpose.getY();

    // Blue-origin field frame: +X points from the blue wall toward red, +Y is the blue driver's
    // left. These signs must be the BLUE-correct mapping, because the alliance flip above already
    // rotates the stick 180 degrees for red. They were previously negated, which made the unflipped
    // case red-correct and left the flip inverting a frame that was already right -- measured with
    // the robot at heading 178 deg, stick-forward commanded vx +0.358 when it needed to be negative.
    teleopSpeeds =
        new ChassisVelocities(
            mappedY * MaxSpeed * teleopSlowModePercent,
            -1.0 * mappedX * MaxSpeed * teleopSlowModePercent,
            rightX * TELEOP_MAX_ANGULAR_RATE * teleopSlowModePercent);

    // Full input chain, for diagnosing direction problems. Order:
    // 0-2 raw controller x/y/theta, 3-5 after deadband+exp+inverts+alliance flip,
    // 6-8 commanded field-relative vx/vy/omega, 9 robot heading deg, 10 isRed.
    SmartDashboard.putNumberArray(
        "Swerve/TeleopDebug",
        new double[] {
          x, y, theta,
          leftX, leftY, rightX,
          teleopSpeeds.vx, teleopSpeeds.vy, teleopSpeeds.omega,
          drivetrainState.Pose.getRotation().getDegrees(),
          FmsSubsystem.isRedAlliance() ? 1.0 : 0.0
        });

    sendSwerveRequest();
  }

  @Override
  protected void collectInputs() {
    if (Utils.isSimulation()) {
      synchronized (simPoseLock) {
        drivetrainState.Pose = simPose;
        fieldRelativeSpeeds = simFieldVelocity;
        double cos = simPose.getRotation().getCos();
        double sin = simPose.getRotation().getSin();
        robotRelativeSpeeds = new ChassisVelocities(
            simFieldVelocity.vx * cos + simFieldVelocity.vy * sin,
            -simFieldVelocity.vx * sin + simFieldVelocity.vy * cos,
            simFieldVelocity.omega);
        drivetrainState.Velocity = robotRelativeSpeeds;
      }
    } else {
      drivetrainState = drivetrain.getState();
      robotRelativeSpeeds = drivetrainState.Velocity;
      fieldRelativeSpeeds = calculateFieldRelativeSpeeds();
    }
    teleopSlowModePercent = ELEVATOR_HEIGHT_TO_SLOW_MODE.get(elevatorHeight);
  }

  public ChassisVelocities getTeleopSpeeds() {
    return teleopSpeeds;
  }

  /**
   * Overrides the field-relative Y (lateral) component of teleop speeds
   * with a specific velocity value, then re-sends the swerve request.
   * Used for trench Y-lock.
   */
  public void overrideTeleopVY(double vyMetersPerSecond) {
    vyOverrideActive = true;
    teleopSpeeds = new ChassisVelocities(
        teleopSpeeds.vx,
        vyMetersPerSecond,
        teleopSpeeds.omega);
    sendSwerveRequest();
  }

  /**
   * Blends the driver's joystick VY with a PID correction VY.
   * blend=0 → full driver control, blend=1 → full PID override.
   */
  public void blendTeleopVY(double pidVy, double blend) {
    vyOverrideActive = true;
    double driverVy = teleopSpeeds.vy;
    double blendedVy = driverVy * (1.0 - blend) + pidVy * blend;
    teleopSpeeds = new ChassisVelocities(
        teleopSpeeds.vx,
        blendedVy,
        teleopSpeeds.omega);
    sendSwerveRequest();
  }

  /**
   * Clears the VY override so normal joystick lateral control resumes immediately.
   */
  public void clearTeleopVYOverride() {
    if (vyOverrideActive) {
      vyOverrideActive = false;
      // Re-send with the original joystick-derived teleopSpeeds (already set by driveTeleop)
      sendSwerveRequest();
    }
  }

  private ChassisVelocities calculateFieldRelativeSpeeds() {
    return robotRelativeSpeeds.toFieldRelative(drivetrainState.Pose.getRotation());
  }

  private void sendSwerveRequest() {
    // If auto speeds were set recently (within 100ms), use them even during teleop
    boolean useAutoSpeeds = !timeSinceAutoSpeeds.hasElapsed(0.1);

    switch (getState()) {
      case TELEOP -> {
        if (useAutoSpeeds) {
          drivetrain.setControl(
              drive
                  .withVelocityX(autoSpeeds.vx)
                  .withVelocityY(autoSpeeds.vy)
                  .withRotationalRate(autoSpeeds.omega)
                  .withDriveRequestType(DriveRequestType.Velocity));
        } else {
          drivetrain.setControl(
              drive
                  .withVelocityX(teleopSpeeds.vx)
                  .withVelocityY(teleopSpeeds.vy)
                  .withRotationalRate(teleopSpeeds.omega)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }
      }
      case TELEOP_SNAPS -> {
        if (teleopSpeeds.omega == 0) {
          drivetrain.setControl(
              driveToAngle
                  .withVelocityX(teleopSpeeds.vx)
                  .withVelocityY(teleopSpeeds.vy)
                  .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                  .withMaxAbsRotationalRate(
                      TELEOP_MAX_ANGULAR_RATE * teleopSlowModePercent)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));

        } else {
          drivetrain.setControl(
              drive
                  .withVelocityX(teleopSpeeds.vx)
                  .withVelocityY(teleopSpeeds.vy)
                  .withRotationalRate(teleopSpeeds.omega)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }
      }
      
      case AUTO ->
          drivetrain.setControl(
              drive
                  .withVelocityX(autoSpeeds.vx)
                  .withVelocityY(autoSpeeds.vy)
                  .withRotationalRate(autoSpeeds.omega)
                  .withDriveRequestType(DriveRequestType.Velocity));
      case AUTO_SNAPS -> {
        drivetrain.setControl(
            driveToAngle
                .withVelocityX(autoSpeeds.vx)
                .withVelocityY(autoSpeeds.vy)
                .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                .withMaxAbsRotationalRate(maxAngularRate)
                .withDriveRequestType(DriveRequestType.Velocity));
      }

    }
  }

  public void normalDriveRequest() {
    if (RobotState.isAutonomous()) {
      setStateFromRequest(SwerveState.AUTO);
    } else {
      setStateFromRequest(SwerveState.TELEOP);
    }
  }

  public Translation2d getControllerValues() {

    var mappedValues =
        ControllerHelpers.fromCircularDiscCoordinates(rawControllerXValue, rawControllerYValue);
    var deadbandX = ControllerHelpers.deadbandJoystickValue(mappedValues.getX(), LEFT_X_DEADBAND);
    var deadbandY = ControllerHelpers.deadbandJoystickValue(mappedValues.getY(), LEFT_Y_DEADBAND);

    return new Translation2d(deadbandX, deadbandY);
  }

  public void snapsDriveRequest(double snapAngle) {
    setSnapToAngle(snapAngle);

    if (RobotState.isAutonomous()) {
      setStateFromRequest(SwerveState.AUTO_SNAPS);
    } else {
      setStateFromRequest(SwerveState.TELEOP_SNAPS);
    }
  }



  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    simNotifier =
        new Notifier(
            () -> {
              double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              drivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
              updateSimPose(deltaTime);
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }

  /**
   * Integrates chassis ground truth for the 2027 alpha desktop stack.
   *
   * <p>Phoenix still simulates each module, but its alpha Pigeon/odometry status stream does not
   * advance reliably on desktop. Feeding an acceleration-limited chassis pose back into Phoenix
   * keeps localization and heading controllers closed-loop until that vendor issue is resolved.
   */
  private void updateSimPose(double deltaTime) {
    ChassisVelocities requested = getRequestedSimFieldVelocity();
    if (!RobotState.isEnabled()) {
      requested = new ChassisVelocities();
    }

    simFieldVelocity = new ChassisVelocities(
        approach(
            simFieldVelocity.vx,
            requested.vx,
            SIM_MAX_TRANSLATIONAL_ACCEL * deltaTime),
        approach(
            simFieldVelocity.vy,
            requested.vy,
            SIM_MAX_TRANSLATIONAL_ACCEL * deltaTime),
        approach(
            simFieldVelocity.omega,
            requested.omega,
            SIM_MAX_ANGULAR_ACCEL * deltaTime));

    synchronized (simPoseLock) {
      simPose = new Pose2d(
          simPose.getX() + simFieldVelocity.vx * deltaTime,
          simPose.getY() + simFieldVelocity.vy * deltaTime,
          simPose.getRotation().plus(
              Rotation2d.fromRadians(simFieldVelocity.omega * deltaTime)));

    }
  }

  private ChassisVelocities getRequestedSimFieldVelocity() {
    ChassisVelocities base = switch (getState()) {
      case AUTO, AUTO_SNAPS -> autoSpeeds;
      case TELEOP, TELEOP_SNAPS ->
          !timeSinceAutoSpeeds.hasElapsed(0.1) ? autoSpeeds : teleopSpeeds;
    };

    boolean facingAngle = getState() == SwerveState.AUTO_SNAPS
        || (getState() == SwerveState.TELEOP_SNAPS && teleopSpeeds.omega == 0.0);
    if (!facingAngle) {
      return base;
    }

    double errorRadians = Math.toRadians(
        frc.robot.AutoMovements.HeadingLockMath.errorDegrees(
            goalSnapAngle, simPose.getRotation().getDegrees()));
    double maxRate = getState() == SwerveState.TELEOP_SNAPS
        ? TELEOP_MAX_ANGULAR_RATE * teleopSlowModePercent
        : maxAngularRate;
    double omega = Math.max(
        -maxRate,
        Math.min(maxRate, ORIGINAL_HEADING_PID.getP() * errorRadians));
    return new ChassisVelocities(base.vx, base.vy, omega);
  }

  private static double approach(double current, double target, double maxDelta) {
    return current + Math.max(-maxDelta, Math.min(maxDelta, target - current));
  }

  /** Resets both Phoenix odometry and desktop-simulation ground truth. */
  public void resetPose(Pose2d pose) {
    synchronized (simPoseLock) {
      simPose = pose;
      simFieldVelocity = new ChassisVelocities();
      drivetrainState.Pose = pose;
      if (!Utils.isSimulation()) {
        drivetrain.resetPose(pose);
      }
    }
  }

  /** Resets heading while preserving translation in both real and simulated odometry. */
  public void resetRotation(Rotation2d rotation) {
    if (Utils.isSimulation()) {
      synchronized (simPoseLock) {
        resetPose(new Pose2d(simPose.getTranslation(), rotation));
      }
    } else {
      drivetrain.resetRotation(rotation);
    }
  }

  public Pose2d getSimPose() {
    synchronized (simPoseLock) {
      return simPose;
    }
  }

  public ChassisVelocities getSimFieldVelocity() {
    synchronized (simPoseLock) {
      return simFieldVelocity;
    }
  }

  /** Approximate drivetrain battery load without blocking on unavailable alpha CAN signals. */
  public double getSimEstimatedSupplyCurrentAmps() {
    synchronized (simPoseLock) {
      double translationFraction = Math.min(1.0, Math.hypot(simFieldVelocity.vx, simFieldVelocity.vy) / MaxSpeed);
      double rotationFraction = Math.min(1.0, Math.abs(simFieldVelocity.omega) / maxAngularRate);
      return 8.0 + 120.0 * translationFraction + 60.0 * rotationFraction;
    }
  }

  /** Stops background native resources created by the drivetrain simulation. */
  public void closeSimulation() {
    if (simNotifier != null) {
      simNotifier.close();
      simNotifier = null;
    }
    drivetrain.close();
  }

  public void setElevatorHeight(double height) {
    elevatorHeight = height;
  }
}
