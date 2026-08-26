package frc.robot.config;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.framework.RobotBase;


public record RobotConfig(String robotName, SwerveConfig swerve, VisionConfig vision) {

  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY) {}

  public record VisionConfig(
      double xyStdDev,
      double thetaStdDev,
      Pose3d robotPoseRelativeToCalibration,
      Pose3d leftBackLimelightPosition,
      Pose3d leftFrontLimelightPosition,
      Pose3d rightLimelightPosition,
      Pose3d gamePieceDetectionLimelightPosition) {}

  public static final boolean IS_DEVELOPMENT = RobotBase.isSimulation() ? true : false;
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  public static RobotConfig get() {
    return CompConfig.competitionBot;
  }
}
