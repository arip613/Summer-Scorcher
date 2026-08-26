package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.driverstation.MatchState;
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

  public void setSnapToAngle(double angle) {
    goalSnapAngle = angle;


    if (RobotState.isAutonomous()) {
      sendSwerveRequest();
    }
  }

  private double elevatorHeight;

  public SwerveSubsystem() {
    super(SubsystemPriority.SWERVE, SwerveState.TELEOP);

    if (Utils.isSimulation()) {
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

    drivetrain.setStateStdDevs(new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002)));
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

    if (MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED) {
      leftX *= -1.0;
      leftY *= -1.0;
    }

    Translation2d mappedpose = ControllerHelpers.fromCircularDiscCoordinates(leftX, leftY);
    double mappedX = mappedpose.getX();
    double mappedY = mappedpose.getY();

    teleopSpeeds =
        new ChassisVelocities(
            -1.0 * mappedY * MaxSpeed * teleopSlowModePercent,
            mappedX * MaxSpeed * teleopSlowModePercent,
            rightX * TELEOP_MAX_ANGULAR_RATE * teleopSlowModePercent);

    sendSwerveRequest();
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
    robotRelativeSpeeds = drivetrainState.Velocity;
    fieldRelativeSpeeds = calculateFieldRelativeSpeeds();
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
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }

  public void setElevatorHeight(double height) {
    elevatorHeight = height;
  }
}
