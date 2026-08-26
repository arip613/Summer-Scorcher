package frc.robot.AutoMovements;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

/** Pure geometry used by the right-trigger passing behavior. */
public final class RightTriggerMath {
  private RightTriggerMath() {}

  public static Translation2d closestPassTarget(
      Translation2d robotTranslation, Translation2d first, Translation2d second) {
    return robotTranslation.getDistance(first) <= robotTranslation.getDistance(second)
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
