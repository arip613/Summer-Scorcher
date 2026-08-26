package frc.robot.AutoMovements;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

class RightTriggerMathTest {
  private static final double EPSILON = 1e-9;

  @Test
  void choosesNearestPassTarget() {
    Translation2d lowTarget = new Translation2d(14.0, 3.5);
    Translation2d highTarget = new Translation2d(14.0, 7.0);
    assertEquals(
        lowTarget,
        RightTriggerMath.closestPassTarget(
            new Translation2d(10.0, 2.0), highTarget, lowTarget));
    assertEquals(
        highTarget,
        RightTriggerMath.closestPassTarget(
            new Translation2d(10.0, 7.5), highTarget, lowTarget));
  }

  @Test
  void aimsAndMeasuresFromShooterPose() {
    Pose2d robotPose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(90.0));
    Transform2d robotToShooter =
        new Transform2d(new Translation2d(1.0, 0.0), Rotation2d.kZero);
    Translation2d target = new Translation2d(2.0, 5.0);

    assertEquals(
        90.0,
        RightTriggerMath.targetHeadingDegrees(robotPose, robotToShooter, target),
        EPSILON);
    assertEquals(
        2.0,
        RightTriggerMath.targetDistanceMeters(robotPose, robotToShooter, target),
        EPSILON);
  }

  @Test
  void shootZoneMirrorsByAlliance() {
    assertTrue(FieldPoints.isInShootZone(12.0, true));
    assertFalse(FieldPoints.isInShootZone(10.0, true));
    assertTrue(FieldPoints.isInShootZone(4.0, false));
    assertFalse(FieldPoints.isInShootZone(6.0, false));
  }

  @Test
  void mirrorsRightAutoPoseToLeftSideAndBack() {
    Pose2d right = new Pose2d(15.7, 5.2, Rotation2d.fromDegrees(230.0));
    Pose2d left = FieldPoints.mirrorPoseLeftRight(right);

    assertEquals(15.7, left.getX(), EPSILON);
    assertEquals(FieldPoints.FIELD_WIDTH - 5.2, left.getY(), EPSILON);
    assertEquals(130.0, left.getRotation().getDegrees(), EPSILON);

    Pose2d roundTrip = FieldPoints.mirrorPoseLeftRight(left);
    assertEquals(right.getX(), roundTrip.getX(), EPSILON);
    assertEquals(right.getY(), roundTrip.getY(), EPSILON);
    assertEquals(right.getRotation().getDegrees(), roundTrip.getRotation().getDegrees(), EPSILON);
  }
}
