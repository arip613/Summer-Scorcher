package frc.robot.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

class VisionRecoveryGateTest {
  @Test
  void acceptsThreeConsistentStrongFrames() {
    VisionRecoveryGate gate = new VisionRecoveryGate();

    assertFalse(gate.evaluate(new Translation2d(2.00, 3.00), 2, 0.08, 0.0, 0.08).ready());
    assertFalse(gate.evaluate(new Translation2d(2.04, 2.98), 2, 0.08, 5.0, 0.07).ready());
    var result = gate.evaluate(new Translation2d(1.98, 3.03), 2, 0.08, 2.0, 0.06);

    assertTrue(result.ready());
    assertEquals(3, result.consistentFrames());
    assertTrue(result.averagedTranslation().getDistance(new Translation2d(2.0, 3.0)) < 0.03);
  }

  @Test
  void allowsStrongCloseSingleTagFrames() {
    VisionRecoveryGate gate = new VisionRecoveryGate();
    assertFalse(gate.evaluate(new Translation2d(1.0, 1.0), 1, 0.03, 0.0, 0.05).ready());
    assertFalse(gate.evaluate(new Translation2d(1.0, 1.0), 1, 0.03, 0.0, 0.05).ready());
    assertTrue(gate.evaluate(new Translation2d(1.0, 1.0), 1, 0.03, 0.0, 0.05).ready());
  }

  @Test
  void rejectsWeakFastStaleAndInconsistentSequences() {
    VisionRecoveryGate gate = new VisionRecoveryGate();
    gate.evaluate(new Translation2d(1.0, 1.0), 2, 0.08, 0.0, 0.05);
    assertEquals(0,
        gate.evaluate(new Translation2d(1.0, 1.0), 1, 0.08, 0.0, 0.05).consistentFrames());

    gate.evaluate(new Translation2d(1.0, 1.0), 2, 0.08, 0.0, 0.05);
    assertEquals(0,
        gate.evaluate(new Translation2d(1.0, 1.0), 2, 0.08, 60.0, 0.05).consistentFrames());

    gate.evaluate(new Translation2d(1.0, 1.0), 2, 0.08, 0.0, 0.05);
    assertEquals(0,
        gate.evaluate(new Translation2d(1.0, 1.0), 2, 0.08, 0.0, 0.75).consistentFrames());

    gate.evaluate(new Translation2d(1.0, 1.0), 2, 0.08, 0.0, 0.05);
    var inconsistent = gate.evaluate(new Translation2d(2.0, 2.0), 2, 0.08, 0.0, 0.05);
    assertEquals(1, inconsistent.consistentFrames());
    assertFalse(inconsistent.ready());
  }
}
