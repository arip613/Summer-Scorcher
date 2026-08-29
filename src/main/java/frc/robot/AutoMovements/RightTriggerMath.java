package frc.robot.AutoMovements;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

/** Pure geometry used by the right-trigger passing behavior. */
public final class RightTriggerMath {
  private RightTriggerMath() {}

  /**
   * Picks the pass target nearer the robot, i.e. the one down the side it is already on.
   *
   * <p>This briefly selected the farther target instead, on the theory that a pass throws the ball
   * across the field. That was wrong, and it only ever looked right because PASS_TARGET_LEFT was
   * set to Y=3.5 -- practically on the centerline (4.035) rather than opposite Y=7.0. With the two
   * targets barely distinguishable, a low-Y robot aimed at mid-field either way. Fixing the
   * coordinate to the true mirror of 7.0 is what fixed the "wrong side" behaviour; the selection
   * rule was fine.
   *
   * <p>Selecting by distance rather than by name also means nothing depends on which target is
   * called "left" and which "right". Those labels do not survive the alliance mirror:
   * mirrorTranslation flips X and keeps Y, so the target named RIGHT is on the red driver's right
   * but the blue driver's left.
   */
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
