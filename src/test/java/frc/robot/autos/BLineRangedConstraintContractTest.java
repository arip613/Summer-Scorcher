package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.RangedConstraint;
import frc.robot.lib.BLine.Path.TranslationTargetConstraint;
import frc.robot.lib.BLine.Path.Waypoint;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Pins the BLine contract that {@link AutoRoutine} relies on to give each waypoint its own speed
 * cap: velocity is resolved per translation ordinal, and because AutoRoutine only ever emits
 * Waypoints, that ordinal is the index of the waypoint. If a BLine upgrade changes how ordinals
 * are assigned, the per-waypoint caps would silently land on the wrong segments -- this catches it.
 */
class BLineRangedConstraintContractTest {
  @Test
  void perWaypointCapsLandOnTheRightWaypoints() {
    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(4.5, 12.0, 200.0, 860.0, 0.03, 2.0, 0.2));

    // Mirrors SideAuto batch 2: caps of 2.7 on waypoints 2, 3, 4 only.
    Double[] caps = {null, null, 2.7, 2.7, 2.7, null, null, null};

    List<Path.PathElement> elements = new ArrayList<>();
    List<RangedConstraint> ranged = new ArrayList<>();
    for (int i = 0; i < caps.length; i++) {
      elements.add(new Waypoint(new Pose2d(i, 0, Rotation2d.kZero)));
      if (caps[i] != null) {
        ranged.add(new RangedConstraint(caps[i], i, i));
      }
    }

    Path path =
        new Path(
            elements,
            new PathConstraints()
                .setMaxVelocityMetersPerSec(ranged.toArray(new RangedConstraint[0])));

    // This is the view FollowPath actually consumes (initialize() calls it).
    var resolved = path.getPathElementsWithConstraintsNoWaypoints();

    List<Double> perWaypointSpeeds = new ArrayList<>();
    for (var pair : resolved) {
      if (pair.getFirst() instanceof Path.TranslationTarget) {
        perWaypointSpeeds.add(
            ((TranslationTargetConstraint) pair.getSecond()).maxVelocityMetersPerSec());
      }
    }

    assertEquals(caps.length, perWaypointSpeeds.size(), "one translation target per waypoint");

    for (int i = 0; i < caps.length; i++) {
      double expected = caps[i] != null ? caps[i] : 4.5;
      double actual = perWaypointSpeeds.get(i);
      System.out.println("waypoint " + i + " -> " + actual + " m/s (expected " + expected + ")");
      assertEquals(expected, actual, 1e-9, "waypoint " + i);
    }
  }
}
