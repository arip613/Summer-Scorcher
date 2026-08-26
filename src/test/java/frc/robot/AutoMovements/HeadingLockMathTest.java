package frc.robot.AutoMovements;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

class HeadingLockMathTest {
  private static final double EPSILON = 1e-9;

  @Test
  void choosesShortestErrorAcrossWraparound() {
    assertEquals(2.0, HeadingLockMath.errorDegrees(-179.0, 179.0), EPSILON);
    assertEquals(-2.0, HeadingLockMath.errorDegrees(179.0, -179.0), EPSILON);
  }

  @Test
  void pointsAtFieldTargetFromRobotPose() {
    Pose2d robotPose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(135.0));
    assertEquals(
        45.0,
        HeadingLockMath.poseTargetDegrees(
            robotPose, Transform2d.kZero, new Translation2d(4.0, 4.0), 0.0),
        EPSILON);
  }

  @Test
  void appliesShooterOffsetAndOperatorOffset() {
    Pose2d robotPose = new Pose2d(1.0, 1.0, Rotation2d.kZero);
    Transform2d robotToShooter =
        new Transform2d(new Translation2d(1.0, 0.0), Rotation2d.kZero);
    assertEquals(
        91.5,
        HeadingLockMath.poseTargetDegrees(
            robotPose, robotToShooter, new Translation2d(2.0, 3.0), 1.5),
        EPSILON);
  }

  @Test
  void convertsLimelightTxToAbsoluteHeading() {
    assertEquals(82.5, HeadingLockMath.visionTargetDegrees(90.0, 9.0, 1.5), EPSILON);
  }

  @Test
  void simulatedTxReconstructsGeometricTargetWithOperatorTrim() {
    double tx = HeadingLockMath.simulatedTxDegrees(100.0, 92.0);
    assertEquals(-8.0, tx, EPSILON);
    assertEquals(101.5, HeadingLockMath.visionTargetDegrees(92.0, tx, 1.5), EPSILON);
  }
}
