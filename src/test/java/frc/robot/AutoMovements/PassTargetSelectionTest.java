package frc.robot.AutoMovements;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

/**
 * End-to-end check of which pass target gets picked, for both alliances.
 *
 * <p>Simulation cannot answer this convincingly: it has no gravity or friction, so the gates that
 * actually blocked passing on the real robot resolve trivially there, and a clean sim run proves
 * only that the geometry is self-consistent. This pins the geometry directly instead.
 *
 * <p>The rule is nearest target -- the pass goes down the side the robot is already on. What was
 * broken was PASS_TARGET_LEFT sitting at Y=3.5, near the centerline, so "nearest" could not tell
 * the two halves apart.
 */
class PassTargetSelectionTest {
  private static final double CENTER_Y = FieldPoints.FIELD_WIDTH / 2.0;

  private static Translation2d pick(double robotX, double robotY, boolean red) {
    Translation2d high = red ? FieldPoints.PASS_TARGET_RIGHT : FieldPoints.PASS_TARGET_RIGHT_BLUE;
    Translation2d low = red ? FieldPoints.PASS_TARGET_LEFT : FieldPoints.PASS_TARGET_LEFT_BLUE;

    return RightTriggerMath.closestPassTarget(new Translation2d(robotX, robotY), high, low);
  }

  /** The pass goes down the robot's own side, so each half picks the target on that half. */
  @Test
  void picksTheTargetOnTheRobotsOwnHalfOnRed() {
    for (double y : new double[] {0.5, 1.7, 3.0, CENTER_Y - 0.2}) {
      assertEquals(FieldPoints.PASS_TARGET_LEFT, pick(10.8, y, true), "red robot at Y=" + y);
    }
    for (double y : new double[] {CENTER_Y + 0.2, 5.5, 6.5, 7.8}) {
      assertEquals(FieldPoints.PASS_TARGET_RIGHT, pick(10.8, y, true), "red robot at Y=" + y);
    }
  }

  @Test
  void picksTheTargetOnTheRobotsOwnHalfOnBlue() {
    for (double y : new double[] {0.5, 1.7, 3.0, CENTER_Y - 0.2}) {
      assertEquals(
          FieldPoints.PASS_TARGET_LEFT_BLUE, pick(5.7, y, false), "blue robot at Y=" + y);
    }
    for (double y : new double[] {CENTER_Y + 0.2, 5.5, 6.5, 7.8}) {
      assertEquals(
          FieldPoints.PASS_TARGET_RIGHT_BLUE, pick(5.7, y, false), "blue robot at Y=" + y);
    }
  }

  /**
   * The exact case from match AZGLE4_Q12: red robot pinned against the low-Y wall. It aimed at
   * Y=3.5 -- nominally the nearer target, but 3.5 was essentially mid-field, which is what made
   * the pass look like it went to the wrong side. With the coordinate corrected it aims at the
   * genuine low-Y target instead.
   */
  @Test
  void theQ12CaseNowAimsDownItsOwnSide() {
    Translation2d picked = pick(10.81, 1.71, true);

    assertEquals(
        FieldPoints.FIELD_WIDTH - 7.0, picked.getY(), 1e-9, "should aim at the true low-Y target");
    assertTrue(picked.getY() < CENTER_Y, "target must be on the robot's own half");
  }

  /** Mirroring must not move a target across the centerline; that would swap the two sides. */
  @Test
  void mirroringKeepsEachTargetOnItsOwnHalf() {
    assertEquals(
        FieldPoints.PASS_TARGET_RIGHT.getY() > CENTER_Y,
        FieldPoints.PASS_TARGET_RIGHT_BLUE.getY() > CENTER_Y,
        "right target changed halves when mirrored to blue");
    assertEquals(
        FieldPoints.PASS_TARGET_LEFT.getY() > CENTER_Y,
        FieldPoints.PASS_TARGET_LEFT_BLUE.getY() > CENTER_Y,
        "left target changed halves when mirrored to blue");
  }
}
