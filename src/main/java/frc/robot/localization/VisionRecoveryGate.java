package frc.robot.localization;

import org.wpilib.math.geometry.Translation2d;

/** Validates a short sequence of vision observations before a large XY recovery reset. */
public final class VisionRecoveryGate {
  public static final int REQUIRED_CONSISTENT_FRAMES = 3;
  public static final int STRONG_MULTI_TAG_COUNT = 2;
  public static final double STRONG_SINGLE_TAG_XY_STD_DEV_M = 0.04;
  public static final double MAX_CONSISTENCY_ERROR_M = 0.20;
  public static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 45.0;
  public static final double MIN_TIMESTAMP_AGE_SEC = -0.05;
  public static final double MAX_TIMESTAMP_AGE_SEC = 0.50;

  public record Evaluation(
      boolean ready, Translation2d averagedTranslation, int consistentFrames, String status) {}

  private Translation2d averagedTranslation = Translation2d.kZero;
  private int consistentFrames;

  public Evaluation evaluate(
      Translation2d translation,
      int tagCount,
      double xyStdDevMeters,
      double angularVelocityDegPerSec,
      double timestampAgeSec) {
    if (timestampAgeSec < MIN_TIMESTAMP_AGE_SEC || timestampAgeSec > MAX_TIMESTAMP_AGE_SEC) {
      return reject("timestamp age");
    }
    if (Math.abs(angularVelocityDegPerSec) > MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
      return reject("angular velocity");
    }
    if (tagCount < STRONG_MULTI_TAG_COUNT
        && xyStdDevMeters > STRONG_SINGLE_TAG_XY_STD_DEV_M) {
      return reject("weak observation");
    }

    if (consistentFrames == 0
        || averagedTranslation.getDistance(translation) > MAX_CONSISTENCY_ERROR_M) {
      averagedTranslation = translation;
      consistentFrames = 1;
      return result(false, "collecting");
    }

    averagedTranslation = averagedTranslation
        .times(consistentFrames)
        .plus(translation)
        .div(consistentFrames + 1.0);
    consistentFrames++;
    return result(consistentFrames >= REQUIRED_CONSISTENT_FRAMES, "consistent");
  }

  public void reset() {
    averagedTranslation = Translation2d.kZero;
    consistentFrames = 0;
  }

  public int getConsistentFrames() {
    return consistentFrames;
  }

  private Evaluation reject(String status) {
    reset();
    return result(false, status);
  }

  private Evaluation result(boolean ready, String status) {
    return new Evaluation(ready, averagedTranslation, consistentFrames, status);
  }
}
