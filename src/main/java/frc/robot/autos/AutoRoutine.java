package frc.robot.autos;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.AutoMovements.FieldPoints;
import frc.robot.imu.BumpCrossingTracker;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.EventTrigger;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.RangedConstraint;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

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
  private final BumpCrossingTracker bumpCrossingTracker;
  private final boolean mirror;
  private final List<Command> steps = new ArrayList<>();
  private Pose2d startPose = null;

  // Accumulated elements for the current BLine path segment, in order. Usually waypoints, but
  // bumpCross() interleaves EventTriggers so it can arm mid-path without splitting the batch.
  private final List<Path.PathElement> pendingElements = new ArrayList<>();
  // Poses backing pendingWaypoints, kept so the batch length can be measured for the timeout.
  private final List<Pose2d> pendingPoses = new ArrayList<>();
  // Per-waypoint max speed constraints (null = use default)
  private final List<Double> pendingMaxSpeeds = new ArrayList<>();
  // Where the previous batch ended, so batch length is measured from the right origin.
  private Pose2d lastFlushedPose = null;

  // Queued parallel commands to run alongside the NEXT drive path
  private final List<Command> pendingParallel = new ArrayList<>();
  private Double pendingTimeoutSeconds = null;

  // Safety timeout: if a drive segment takes longer than this, skip it and move on.
  // Prevents the entire auto from stalling if the robot gets stuck or pose is wrong.
  //
  // This must scale with the batch. flushPendingPath() collapses every consecutive driveTo into
  // ONE BLine path, so a flat timeout is a budget for the whole batch rather than for one hop --
  // the 8s flat value was shorter than the ~14m second half of SideAuto takes to drive, which cut
  // the path off partway and dropped the robot wherever it happened to be when the clock ran out.
  private static final double DRIVE_TIMEOUT_BASE_SECS = 3.0;
  private static final double DRIVE_TIMEOUT_ASSUMED_SPEED_MPS = 1.5;
  private static final double DRIVE_TIMEOUT_MAX_SECS = 15.0;

  // BLine's event trigger registry is static, so keys must be unique across every routine built.
  private static int nextBumpCrossId = 0;


  private AutoRoutine(SwerveSubsystem swerve, LocalizationSubsystem localization,
                      FollowPath.Builder pathBuilder, BumpCrossingTracker bumpCrossingTracker,
                      boolean mirror) {
    this.swerve = swerve;
    this.localization = localization;
    this.pathBuilder = pathBuilder;
    this.bumpCrossingTracker = bumpCrossingTracker;
    this.mirror = mirror;
  }

  /** Create a new auto routine builder using BLine path following. */
  public static AutoRoutine create(SwerveSubsystem swerve, LocalizationSubsystem localization,
                                   FollowPath.Builder pathBuilder,
                                   BumpCrossingTracker bumpCrossingTracker) {
    return new AutoRoutine(swerve, localization, pathBuilder, bumpCrossingTracker, false);
  }

  /** Create a new auto routine builder that mirrors all poses (red → blue). */
  public static AutoRoutine createMirrored(SwerveSubsystem swerve,
                                           LocalizationSubsystem localization,
                                           FollowPath.Builder pathBuilder,
                                           BumpCrossingTracker bumpCrossingTracker) {
    return new AutoRoutine(swerve, localization, pathBuilder, bumpCrossingTracker, true);
  }

  // ---- Starting pose ----

  public AutoRoutine startAt(Pose2d pose) {
    this.startPose = maybeMirror(pose);
    this.lastFlushedPose = this.startPose;
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
    pendingElements.add(new Waypoint(pose));
    pendingPoses.add(pose);
    pendingMaxSpeeds.add(maxSpeed);
  }

  // ---- Parallel commands ----

  public AutoRoutine doWhileDriving(Command cmd) {
    pendingParallel.add(cmd);
    return this;
  }

  public AutoRoutine withTimeout(double seconds) {
    if (seconds <= 0) return this;
    if (!steps.isEmpty() && pendingPoses.isEmpty()) {
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

  // ---- Bump crossing ----

  /**
   * Arms a bump crossing for the next driveTo. While the IMU reports the robot tilted along
   * {@code crossingDirection}, the drive commits to a fixed speed across the bump instead of
   * tracking the path, because the pose estimate is not trustworthy mid-bump.
   *
   * <p>This does NOT split the path. It inserts a BLine EventTrigger that fires when the robot
   * enters the following segment, so the preceding waypoint stays an intermediate handoff. Adding
   * this as an ordinary sequential step instead would make that waypoint a batch endpoint, forcing
   * a full settle to the 3cm end tolerance and arriving at the bump from a standstill -- both slow
   * and exactly the wrong way to take a bump.
   *
   * @param crossingDirection field-relative direction of travel across the bump, given in RED
   *     coordinates like every other pose here; it is mirrored for blue automatically
   */
  public AutoRoutine bumpCross(Rotation2d crossingDirection) {
    return bumpCross(crossingDirection, null);
  }

  /**
   * Arms a bump crossing that also re-localizes on landing.
   *
   * @param crossingDirection field-relative direction of travel, in RED coordinates
   * @param landingPoint where the robot physically ends up once flat on the far side, in RED
   *     coordinates. This hard-resets the pose translation, so a wrong value is worse than none --
   *     measure it, do not infer it from the path waypoints.
   */
  public AutoRoutine bumpCross(Rotation2d crossingDirection, Translation2d landingPoint) {
    // mirrorPose reflects across field center X and maps heading to 180 - heading, so the crossing
    // direction has to go through the same transform rather than being used as authored.
    Rotation2d direction =
        mirror
            ? FieldPoints.mirrorPose(new Pose2d(Translation2d.kZero, crossingDirection))
                .getRotation()
            : crossingDirection;
    Optional<Translation2d> landing =
        landingPoint == null
            ? Optional.empty()
            : Optional.of(
                mirror
                    ? FieldPoints.mirrorPose(new Pose2d(landingPoint, Rotation2d.kZero))
                        .getTranslation()
                    : landingPoint);

    Runnable arm = () -> bumpCrossingTracker.bumpCrossRequest(direction, landing);

    if (pendingPoses.isEmpty()) {
      // Nothing to hang a trigger off of -- no preceding waypoint means no segment to fire on.
      // Fall back to arming as a plain sequential step.
      steps.add(Commands.runOnce(arm).withName("ArmBumpCrossing"));
      return this;
    }

    // The registry is static and shared across every routine built this session, so the key has to
    // be unique per crossing rather than per auto.
    String key = "AutoRoutine/BumpCross/" + (nextBumpCrossId++);
    FollowPath.registerEventTrigger(key, arm);

    // t_ratio 0 fires as soon as the robot enters the segment that starts at the waypoint just
    // added, which is the segment that crosses the bump.
    pendingElements.add(new EventTrigger(0.0, key));

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
    if (pendingPoses.isEmpty()) {
      return;
    }

    List<Path.PathElement> elements = new ArrayList<>(pendingElements);
    Path path;

    // Apply each waypoint's speed cap to that waypoint alone, using BLine's ranged constraints.
    // Path resolves velocity per element ordinal, and because this builder only ever emits
    // Waypoints, the translation ordinal is just the index into pendingWaypoints. The constraint
    // at index i governs the approach TO waypoint i, which is what driveToAll(..., maxSpeed)
    // means. Collapsing the batch to its single most restrictive cap instead -- as this used to
    // do -- slowed the whole path down: in SideAuto only 5.2m of the 16.5m second batch asked
    // for 2.7 m/s, but all of it ran at 2.7 m/s, costing about 1.4s of a 20s auto.
    List<RangedConstraint> speedConstraints = new ArrayList<>();
    for (int i = 0; i < pendingMaxSpeeds.size(); i++) {
      Double speed = pendingMaxSpeeds.get(i);
      if (speed != null) {
        speedConstraints.add(new RangedConstraint(speed, i, i));
      }
    }

    if (speedConstraints.isEmpty()) {
      path = new Path(elements);
    } else {
      PathConstraints constraints = new PathConstraints()
          .setMaxVelocityMetersPerSec(speedConstraints.toArray(new RangedConstraint[0]));
      path = new Path(elements, constraints);
    }

    // The safety timeout stays keyed to the slowest cap in the batch: it is a stuck-robot escape
    // hatch, so the conservative estimate is the correct one.
    Double minMaxSpeed = null;
    for (Double speed : pendingMaxSpeeds) {
      if (speed != null && (minMaxSpeed == null || speed < minMaxSpeed)) {
        minMaxSpeed = speed;
      }
    }

    Command drive = pathBuilder.build(path);
    double timeout = pendingTimeoutSeconds != null
        ? pendingTimeoutSeconds
        : safetyTimeoutFor(pendingPathLengthMeters(), minMaxSpeed);

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

    if (!pendingPoses.isEmpty()) {
      lastFlushedPose = pendingPoses.get(pendingPoses.size() - 1);
    }

    pendingElements.clear();
    pendingPoses.clear();
    pendingMaxSpeeds.clear();
  }

  /**
   * Straight-line length of the batch about to be flushed, measured from wherever the previous
   * batch ended. BLine drives waypoint-to-waypoint in straight segments, so this is a good
   * estimate of the distance the robot actually covers.
   */
  private double pendingPathLengthMeters() {
    double length = 0.0;
    Pose2d previous = lastFlushedPose;

    for (Pose2d pose : pendingPoses) {
      if (previous != null) {
        length += previous.getTranslation().getDistance(pose.getTranslation());
      }
      previous = pose;
    }

    return length;
  }

  /**
   * Safety timeout for a drive batch of the given length. Generous on purpose: this is a
   * stuck-robot escape hatch, not a schedule. If it fires during a normal run the path is cut off
   * partway and the rest of the auto continues from an unplanned pose, which is far worse than
   * simply arriving late.
   */
  private static double safetyTimeoutFor(double pathLengthMeters, Double batchMaxSpeed) {
    double assumedSpeed = batchMaxSpeed != null
        ? Math.min(batchMaxSpeed, DRIVE_TIMEOUT_ASSUMED_SPEED_MPS)
        : DRIVE_TIMEOUT_ASSUMED_SPEED_MPS;

    double estimate = DRIVE_TIMEOUT_BASE_SECS + (pathLengthMeters / assumedSpeed);

    return Math.min(estimate, DRIVE_TIMEOUT_MAX_SECS);
  }
}
