package frc.robot.vision.results;

import org.wpilib.math.linalg.Vector;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.numbers.N3;
import frc.robot.util.ReusableOptional;

/**
 * Optional wrapper for TagResult that can be reused to reduce allocations.
 */
public class OptionalTagResult extends ReusableOptional<TagResult> {
  public OptionalTagResult() {
    // Initialize with a harmless default; will be replaced on update.
    super(new TagResult(new Pose2d(), 0.0, null, 0));
  }

  /** Clears the value and marks as empty. */
  public OptionalTagResult empty() {
    this.isPresent = false;
    return this;
  }

  /** Updates the value and marks as present. */
  public OptionalTagResult update(
      Pose2d pose,
      double timestampSeconds,
      Vector<N3> stdDevs,
      int tagCount) {
    this.value = new TagResult(pose, timestampSeconds, stdDevs, tagCount);
    this.isPresent = true;
    return this;
  }
}
