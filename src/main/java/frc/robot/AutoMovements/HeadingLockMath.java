package frc.robot.AutoMovements;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

/** Pure heading-lock geometry, separated from hardware so it can be regression tested. */
public final class HeadingLockMath {
  private HeadingLockMath() {}

  /** Returns the shortest signed angular error in degrees in the range [-180, 180). */
  public static double errorDegrees(double targetDegrees, double currentDegrees) {
    return ((targetDegrees - currentDegrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
  }

  /** Calculates the robot heading that points the robot-relative shooter at a field target. */
  public static double poseTargetDegrees(
      Pose2d robotPose,
      Transform2d robotToShooter,
      Translation2d targetPoint,
      double operatorOverrideDegrees) {
    Translation2d shooter = robotPose.transformBy(robotToShooter).getTranslation();
    double dx = targetPoint.getX() - shooter.getX();
    double dy = targetPoint.getY() - shooter.getY();

    if (Math.hypot(dx, dy) < 1e-6) {
      return robotPose.getRotation().getDegrees();
    }
    return Math.toDegrees(Math.atan2(dy, dx)) + operatorOverrideDegrees;
  }

  /** Converts Limelight horizontal error into an absolute field heading. */
  public static double visionTargetDegrees(
      double currentHeadingDegrees, double txDegrees, double operatorOverrideDegrees) {
    return currentHeadingDegrees - txDegrees + operatorOverrideDegrees;
  }

  /** Produces ideal Limelight tx for simulation using the same sign convention as vision aiming. */
  public static double simulatedTxDegrees(
      double geometricTargetDegrees, double currentHeadingDegrees) {
    return -errorDegrees(geometricTargetDegrees, currentHeadingDegrees);
  }
}
