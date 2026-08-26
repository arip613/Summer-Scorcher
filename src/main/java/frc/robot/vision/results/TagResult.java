package frc.robot.vision.results;

import org.wpilib.math.linalg.Vector;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.numbers.N3;

/**
 * Minimal value type representing a single AprilTag vision measurement.
 */
public class TagResult {
  private final Pose2d pose;
  private final double timestampSeconds;
  private final Vector<N3> standardDevs;
  private final int tagCount;

  public TagResult(Pose2d pose, double timestampSeconds, Vector<N3> standardDevs, int tagCount) {
    this.pose = pose;
    this.timestampSeconds = timestampSeconds;
    this.standardDevs = standardDevs;
    this.tagCount = tagCount;
  }

  public Pose2d pose() {
    return pose;
  }

  public double timestamp() {
    return timestampSeconds;
  }

  public Vector<N3> standardDevs() {
    return standardDevs;
  }

  public int tagCount() {
    return tagCount;
  }
}
