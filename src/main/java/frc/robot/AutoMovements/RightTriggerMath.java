package frc.robot.AutoMovements;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

/** Pure geometry used by the right-trigger passing behavior. */
public final class RightTriggerMath {
  private RightTriggerMath() {}

  /**
   * Picks the pass target further from the robot, which is the one across the field.
   *
   * <p>This used to pick the nearer one, which threw the ball to the side the robot was already
   * standing on. In match AZGLE4_Q12 the robot sat at Y=1.70, hard against the low-Y wall, and
   * aimed at the target at Y=3.5 -- 2.5m nearer than the one at Y=7.0, and the wrong way across
   * the field.
   *
   * <p>Selecting by distance rather than by name also keeps this correct regardless of which
   * target is called "left" and which "right". Those labels do not survive the alliance mirror:
   * mirrorTranslation flips X and keeps Y, so the target named RIGHT is on the red driver's right
   * but on the blue driver's left. Nothing here depends on that, and nothing else should either.
   */
  public static Translation2d farthestPassTarget(
      Translation2d robotTranslation, Translation2d first, Translation2d second) {
    return robotTranslation.getDistance(first) >= robotTranslation.getDistance(second)
        ? first
        : second;
  }

  public static double targetHeadingDegrees(
      Pose2d robotPose, Transform2d robotToShooter, Translation2d target) {
    return HeadingLockMath.poseTargetDegrees(robotPose, robotToShooter, target, 0.0);
  }

  public static double targetDistanceMeters(
      Pose2d robotPose, Transform2d robotToShooter, Translation2d target) {
    return robotPose.transformBy(robotToShooter).getTranslation().getDistance(target);
  }
}
