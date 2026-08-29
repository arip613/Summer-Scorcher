package frc.robot.imu;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Covers the tilt projection, which is the part of bump detection most likely to be subtly wrong.
 * Raw pitch is only meaningful when the robot drives along its own X axis; SideAuto crosses the
 * bump at heading 270, where climbing shows up almost entirely as roll.
 */
class BumpCrossingTrackerTest {
  private static final double TOL = 1e-6;

  private static double tilt(
      double pitchDeg, double rollDeg, double headingDeg, double crossingDeg) {
    return BumpCrossingTracker.calculateDirectionalTilt(
        pitchDeg, rollDeg, headingDeg, Rotation2d.fromDegrees(crossingDeg));
  }

  @Test
  void levelRobotIsNeverTilted() {
    for (double heading = 0; heading < 360; heading += 45) {
      assertEquals(0.0, tilt(0, 0, heading, 0), TOL, "heading " + heading);
    }
  }

  @Test
  void noseUpDrivingAlongCrossingDirectionReadsAsUphill() {
    // Facing +X, crossing +X, pitched nose-up 10 deg.
    assertEquals(10.0, tilt(10, 0, 0, 0), 1e-4);
  }

  @Test
  void noseDownDrivingAlongCrossingDirectionReadsAsDownhill() {
    assertEquals(-10.0, tilt(-10, 0, 0, 0), 1e-4);
  }

  @Test
  void climbingSidewaysAtHeading270ShowsUpAsRollNotPitch() {
    // This is SideAuto's case: robot held at 270 while crossing toward +X. A robot climbing the
    // near face here has ~zero pitch, so a raw-pitch check would see nothing.
    double rollOnly = tilt(0.0, 10.0, 270.0, 0.0);

    assertTrue(
        Math.abs(rollOnly) > 9.0,
        "10 deg of roll at heading 270 should register as a real crossing tilt, got " + rollOnly);
  }

  @Test
  void tiltSignFlipsWithCrossingDirection() {
    double forward = tilt(10, 0, 0, 0);
    double reversed = tilt(10, 0, 0, 180);

    assertEquals(-forward, reversed, 1e-4, "same physical tilt, opposite direction of travel");
  }

  @Test
  void tiltPerpendicularToTravelIsIgnored() {
    // Facing +X and crossing +X, but rolled sideways: that is across the direction of travel and
    // must not read as climbing, or side-to-side rocking would trip the state machine.
    assertEquals(0.0, tilt(0, 12.0, 0, 0), 1e-4);
  }
}
