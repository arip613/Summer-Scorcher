package frc.robot.AutoMovements;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.Trajectory;
import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import java.util.Optional;
import java.util.function.Supplier;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;

/**
 * Command that follows a WPILib Trajectory using PID controllers.
 * Adapted from Littleton Robotics / Mechanical Advantage.
 */
public class DriveTrajectory extends Command {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private static double linearkP = 8.0;
  private static double linearkD = 0.0;
  private static double thetakP = 4.0;
  private static double thetakD = 0.0;

  static {
    SmartDashboard.putNumber("DriveTrajectory/LinearkP", linearkP);
    SmartDashboard.putNumber("DriveTrajectory/LinearkD", linearkD);
    SmartDashboard.putNumber("DriveTrajectory/ThetakP", thetakP);
    SmartDashboard.putNumber("DriveTrajectory/ThetakD", thetakD);
  }

  private final Timer timer = new Timer();
  private final Trajectory trajectory;
  private final Supplier<Optional<Double>> omegaOverride;
  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  public DriveTrajectory(
      Trajectory trajectory,
      Supplier<Optional<Double>> omegaOverride,
      SwerveSubsystem swerve,
      LocalizationSubsystem localization) {
    this.swerve = swerve;
    this.localization = localization;
    this.trajectory = trajectory;
    this.omegaOverride = omegaOverride;
    xController = new PIDController(linearkP, 0, linearkD, LOOP_PERIOD_SECS);
    yController = new PIDController(linearkP, 0, linearkD, LOOP_PERIOD_SECS);
    thetaController = new PIDController(thetakP, 0, thetakD, LOOP_PERIOD_SECS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerve);
  }

  public DriveTrajectory(Trajectory trajectory, SwerveSubsystem swerve, LocalizationSubsystem localization) {
    this(trajectory, () -> Optional.empty(), swerve, localization);
  }

  @Override
  public void initialize() {
    timer.restart();

    linearkP = SmartDashboard.getNumber("DriveTrajectory/LinearkP", linearkP);
    linearkD = SmartDashboard.getNumber("DriveTrajectory/LinearkD", linearkD);
    thetakP = SmartDashboard.getNumber("DriveTrajectory/ThetakP", thetakP);
    thetakD = SmartDashboard.getNumber("DriveTrajectory/ThetakD", thetakD);

    xController.setPID(linearkP, 0, linearkD);
    yController.setPID(linearkP, 0, linearkD);
    thetaController.setPID(thetakP, 0, thetakD);

    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {
    Pose2d currentPose = localization.getPose();
    Trajectory.State desiredState = trajectory.sample(timer.get());
    Pose2d desiredPose = desiredState.pose;

    double xOutput =
        xController.calculate(currentPose.getX(), desiredPose.getX())
            + desiredState.velocity * desiredPose.getRotation().getCos();
    double yOutput =
        yController.calculate(currentPose.getY(), desiredPose.getY())
            + desiredState.velocity * desiredPose.getRotation().getSin();
    double thetaOutput =
        omegaOverride.get().orElseGet(
            () -> thetaController.calculate(
                currentPose.getRotation().getRadians(),
                desiredPose.getRotation().getRadians()));

    swerve.setFieldRelativeAutoSpeeds(
        new ChassisVelocities(xOutput, yOutput, thetaOutput)
            .toRobotRelative(currentPose.getRotation()));

    SmartDashboard.putNumber("DriveTrajectory/TranslationError",
        currentPose.getTranslation().getDistance(desiredPose.getTranslation()));
    SmartDashboard.putNumber("DriveTrajectory/RotationError",
        currentPose.getRotation().minus(desiredPose.getRotation()).getDegrees());
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTime());
  }

  @Override
  public void end(boolean interrupted) {
    // Let default swerve command take over
  }
}
