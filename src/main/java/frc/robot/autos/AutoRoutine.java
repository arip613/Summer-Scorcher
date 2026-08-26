package frc.robot.autos;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.AutoMovements.FieldPoints;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.lib.BLine.Path.PathConstraints;

import java.util.ArrayList;
import java.util.List;

/**
 * A fluent builder for creating autonomous routines using BLine path following.
 *
 * Consecutive driveTo/driveToAll calls are batched into a single BLine Path
 * for smooth multi-waypoint following. Actions (run, waitSeconds, etc.) flush
 * the accumulated waypoints into a path command before adding the action.
 *
 * Usage example:
 * <pre>
 *   Command auto = AutoRoutine.create(swerve, localization, pathBuilder)
 *       .startAt(14.0, 7.0, 180.0)
 *       .driveToAll(12.0, 7.0, 180.0)
 *       .driveToAll(11.0, 6.0, 200.0)    // batched into one smooth path
 *       .run(shootCommand())              // flushes path, then runs action
 *       .driveToAll(10.0, 5.0, 180.0)
 *       .build();
 * </pre>
 */
public class AutoRoutine {
  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final FollowPath.Builder pathBuilder;
  private final boolean mirror;
  private final List<Command> steps = new ArrayList<>();
  private Pose2d startPose = null;

  // Accumulated waypoints for the current BLine path segment.
  private final List<Waypoint> pendingWaypoints = new ArrayList<>();
  // Per-waypoint max speed constraints (null = use default)
  private final List<Double> pendingMaxSpeeds = new ArrayList<>();

  // Queued parallel commands to run alongside the NEXT drive path
  private final List<Command> pendingParallel = new ArrayList<>();
  private Double pendingTimeoutSeconds = null;

  // Safety timeout: if a drive segment takes longer than this, skip it and move on.
  // Prevents the entire auto from stalling if the robot gets stuck or pose is wrong.
  private static final double DRIVE_SAFETY_TIMEOUT_SECS = 8.0;


  private AutoRoutine(SwerveSubsystem swerve, LocalizationSubsystem localization,
                      FollowPath.Builder pathBuilder, boolean mirror) {
    this.swerve = swerve;
    this.localization = localization;
    this.pathBuilder = pathBuilder;
    this.mirror = mirror;
  }

  /** Create a new auto routine builder using BLine path following. */
  public static AutoRoutine create(SwerveSubsystem swerve, LocalizationSubsystem localization,
                                   FollowPath.Builder pathBuilder) {
    return new AutoRoutine(swerve, localization, pathBuilder, false);
  }

  /** Create a new auto routine builder that mirrors all poses (red → blue). */
  public static AutoRoutine createMirrored(SwerveSubsystem swerve,
                                           LocalizationSubsystem localization,
                                           FollowPath.Builder pathBuilder) {
    return new AutoRoutine(swerve, localization, pathBuilder, true);
  }

  // ---- Starting pose ----

  public AutoRoutine startAt(Pose2d pose) {
    this.startPose = maybeMirror(pose);
    return this;
  }

  public AutoRoutine startAt(double x, double y, double headingDeg) {
    return startAt(new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg)));
  }

  // ---- Drive steps (accumulated into BLine paths) ----

  public AutoRoutine driveTo(Pose2d target) {
    addWaypoint(maybeMirror(target), null);
    return this;
  }

  public AutoRoutine driveTo(double x, double y, double headingDeg) {
    return driveTo(new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg)));
  }

  public AutoRoutine driveToAll(Pose2d target) {
    addWaypoint(maybeMirror(target), null);
    return this;
  }

  public AutoRoutine driveToAll(double x, double y, double headingDeg) {
    return driveToAll(new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg)));
  }

  public AutoRoutine driveToAll(double x, double y, double headingDeg, double maxSpeed) {
    addWaypoint(maybeMirror(new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg))), maxSpeed);
    return this;
  }

  public AutoRoutine driveToAll(Pose2d target, double maxSpeed) {
    addWaypoint(maybeMirror(target), maxSpeed);
    return this;
  }

  private void addWaypoint(Pose2d pose, Double maxSpeed) {
    pendingWaypoints.add(new Waypoint(pose));
    pendingMaxSpeeds.add(maxSpeed);
  }

  // ---- Parallel commands ----

  public AutoRoutine doWhileDriving(Command cmd) {
    pendingParallel.add(cmd);
    return this;
  }

  public AutoRoutine withTimeout(double seconds) {
    if (seconds <= 0) return this;
    if (!steps.isEmpty() && pendingWaypoints.isEmpty()) {
      int lastIndex = steps.size() - 1;
      Command last = steps.remove(lastIndex);
      steps.add(last.withTimeout(seconds));
    } else {
      pendingTimeoutSeconds = seconds;
    }
    return this;
  }

  // ---- Sequential action steps ----

  public AutoRoutine run(Command cmd) {
    flushPendingPath();
    steps.add(cmd);
    return this;
  }

  public AutoRoutine runOnce(Runnable action) {
    flushPendingPath();
    steps.add(Commands.runOnce(action));
    return this;
  }

  public AutoRoutine runFor(double seconds, Command cmd) {
    flushPendingPath();
    steps.add(cmd.withTimeout(seconds));
    return this;
  }

  public AutoRoutine waitSeconds(double seconds) {
    flushPendingPath();
    steps.add(Commands.waitSeconds(seconds));
    return this;
  }

  public AutoRoutine runUntil(java.util.function.BooleanSupplier condition, Command cmd) {
    flushPendingPath();
    steps.add(cmd.until(condition));
    return this;
  }

  // ---- Build ----

  public Command build() {
    flushPendingPath();

    List<Command> allSteps = new ArrayList<>();

    if (startPose != null) {
      final Pose2d pose = startPose;
      allSteps.add(Commands.runOnce(() -> {
        localization.resetPose(pose);
      }).withName("ResetPose"));
    }

    allSteps.addAll(steps);

    allSteps.add(Commands.runOnce(() -> {
      swerve.setFieldRelativeAutoSpeeds(new ChassisVelocities());
    }).withName("StopDrive"));

    return Commands.sequence(allSteps.toArray(new Command[0])).withName("AutoRoutine");
  }

  // ---- Internal helpers ----

  private Pose2d maybeMirror(Pose2d pose) {
    if (!mirror) {
      return pose;
    }
    return FieldPoints.mirrorPose(pose);
  }

  /**
   * Flush accumulated waypoints into a single BLine FollowPath command.
   * If there are pending parallel commands, they run alongside the path.
   */
  private void flushPendingPath() {
    if (pendingWaypoints.isEmpty()) {
      return;
    }

    List<Path.PathElement> elements = new ArrayList<>(pendingWaypoints);
    Path path;

    // A BLine constraint applies to this complete batched path, so use the most restrictive
    // maximum requested by any waypoint in the segment.
    Double minMaxSpeed = null;
    for (Double speed : pendingMaxSpeeds) {
      if (speed != null && (minMaxSpeed == null || speed < minMaxSpeed)) {
        minMaxSpeed = speed;
      }
    }

    if (minMaxSpeed != null) {
      PathConstraints constraints = new PathConstraints()
          .setMaxVelocityMetersPerSec(minMaxSpeed);
      path = new Path(elements, constraints);
    } else {
      path = new Path(elements);
    }

    Command drive = pathBuilder.build(path);
    double timeout = pendingTimeoutSeconds != null
        ? pendingTimeoutSeconds
        : DRIVE_SAFETY_TIMEOUT_SECS;

    pendingTimeoutSeconds = null;

    // Attach parallel commands if any
    if (!pendingParallel.isEmpty()) {
      List<Command> parallel = new ArrayList<>(pendingParallel);
      pendingParallel.clear();
      Command combined = drive;
      for (Command cmd : parallel) {
        combined = combined.alongWith(cmd);
      }
      steps.add(combined.withTimeout(timeout).withName("BLineDriveWithParallel"));
    } else {
      steps.add(drive.withTimeout(timeout).withName("BLineDrive"));
    }

    pendingWaypoints.clear();
    pendingMaxSpeeds.clear();
  }
}
