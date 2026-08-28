package frc.robot.config;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.VisionConfig;

class CompConfig {
  private static PhoenixPIDController createSnapController() {
    // P=4.5, D=0. A D term (tried 0.30) was reverted: heading now derives from real camera tx,
    // and D amplified that signal's jitter into twitchy rotation -- blocked frames on the heading
    // gate went from 0 to 89. The overshoot it was meant to damp is better handled by the widened
    // HEADING_TOLERANCE_DEG and settle time in HeadingLock.
    var controller = new PhoenixPIDController(4.5, 0.00, 0.0);
    controller.setIZone(8.0);
    return controller;
  }

  public static final RobotConfig competitionBot =
                  new RobotConfig(
                      "comp",
                      new SwerveConfig(createSnapController(), true, false, false),
                      new VisionConfig(
                          0.005,
                          0.8,
                          // All camera offsets handled in Limelight UI; leave code poses at identity.
                          new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)),
                          new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)),
                          new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)),
                          new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)),
                          new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0))));

  private CompConfig() {}
}
