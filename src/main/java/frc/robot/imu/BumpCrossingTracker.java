package frc.robot.imu;

import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.util.Optional;
import java.util.function.Consumer;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.smartdashboard.SmartDashboard;

/**
 * Detects a bump crossing from IMU tilt and commits the drive to a fixed speed across it.
 *
 * <p>Adapted from FRC 581's BumpCrossingTracker/BumpCrossingFollower. The premise is that a closed
 * loop path follower should not be trusted mid-bump: the wheels unload, odometry over-reports, and
 * the follower reacts to a pose that is briefly fiction. Rather than tune through that, the drive
 * commits to a constant speed along the crossing direction for the duration of the crossing while
 * still correcting laterally and holding heading, and optionally re-localizes on landing.
 */
public class BumpCrossingTracker extends StateMachine<BumpCrossingState> {
  private static final double FLAT_THRESHOLD_DEGREES = 5.0;
  private static final double UPHILL_THRESHOLD_DEGREES = 5.0;
  private static final double DOWNHILL_THRESHOLD_DEGREES = -5.0;

  private static final double FLAT_DEBOUNCE_SECONDS = 0.1;
  private static final double FLAT_FALLBACK_DEBOUNCE_SECONDS = 0.75;
  /**
   * Bounds on how long the drive override may stay engaged, per phase. These are deliberately
   * tight: the override is open loop along the crossing direction, so a phase that never completes
   * is a robot driving blind. The approach phase is the dangerous one -- if the IMU tilt sign is
   * opposite what this expects, the climb is never detected and only this timeout stops it. At 3
   * m/s a 1.5s approach is 4.5m, already longer than the 3.5m crossing segment in SideAuto.
   */
  private static final double APPROACH_TIMEOUT_SECONDS = 1.5;

  private static final double ON_BUMP_TIMEOUT_SECONDS = 1.5;

  /**
   * Speed commanded along the crossing direction while on the bump, matching 581.
   *
   * <p>This must not be set below what the follower would have commanded anyway, or the override
   * becomes a brake. BLine drives on remaining path distance, so mid-crossing it is asking for
   * P * ~2m = 5 m/s, clamped to the 4.5 m/s path limit. Anything under that slows the crossing
   * down instead of committing to it.
   *
   * <p>4.0 rather than the full 4.5 leaves vector headroom: the perpendicular cross-track term is
   * added on top of this and nothing re-clamps the result before it reaches the modules, so
   * commanding the limit along the crossing direction would squeeze out the correction keeping the
   * robot centered on the bump.
   */
  private static final double CROSSING_LINEAR_VELOCITY = 4.0;

  /**
   * Signed tilt along the direction of travel, in degrees. Positive means tilted up toward the
   * crossing direction.
   *
   * <p>Pitch and roll are robot frame while the crossing direction is field frame, so raw pitch is
   * only meaningful when driving along the robot's own X axis. Rotating the body-up unit vector by
   * the robot's full orientation and then into the crossing frame gives a tilt that is correct at
   * any heading -- which matters here because the bump is crossed at heading 270.
   */
  public static double calculateDirectionalTilt(
      double pitchDegrees,
      double rollDegrees,
      double headingDegrees,
      Rotation2d crossingDirection) {
    var robotOrientation =
        new Rotation3d(
            Math.toRadians(rollDegrees),
            Math.toRadians(pitchDegrees),
            Math.toRadians(headingDegrees));
    var bodyUpInCrossingFrame =
        new Translation3d(0.0, 0.0, 1.0)
            .rotateBy(robotOrientation)
            .rotateBy(new Rotation3d(0.0, 0.0, -crossingDirection.getRadians()));

    return Math.toDegrees(Math.asin(bodyUpInCrossingFrame.getX()));
  }

  private final ImuSubsystem imu;
  private final Consumer<Translation2d> poseResetConsumer;

  private final Debouncer flatDebouncer = new Debouncer(FLAT_DEBOUNCE_SECONDS, DebounceType.kRising);
  private final Debouncer flatFallbackDebouncer =
      new Debouncer(FLAT_FALLBACK_DEBOUNCE_SECONDS, DebounceType.kRising);

  private Rotation2d crossingDirection = Rotation2d.kZero;
  private Optional<Translation2d> landingPoint = Optional.empty();
  private double directionalTilt = 0.0;
  private boolean isFlat = true;
  private boolean isFlatFallbackDebounced = false;
  private int completedCrossings = 0;

  public BumpCrossingTracker(ImuSubsystem imu, Consumer<Translation2d> poseResetConsumer) {
    super(SubsystemPriority.BUMP_CROSSING, BumpCrossingState.FLAT_NOT_CROSSING);
    this.imu = imu;
    this.poseResetConsumer = poseResetConsumer;
  }

  /**
   * Arms a crossing. Call this immediately before the drive segment that crosses the bump.
   *
   * @param crossingDirection field-relative direction of travel across the bump
   */
  public void bumpCrossRequest(Rotation2d crossingDirection) {
    bumpCrossRequest(crossingDirection, Optional.empty());
  }

  /**
   * Arms a crossing that also re-localizes on landing.
   *
   * @param crossingDirection field-relative direction of travel across the bump
   * @param landingPoint where the robot physically ends up once it is flat on the far side. This
   *     hard-resets the pose translation, so a wrong value is worse than no value -- measure it
   *     rather than guessing from the path waypoints.
   */
  public void bumpCrossRequest(
      Rotation2d crossingDirection, Optional<Translation2d> landingPoint) {
    this.crossingDirection = crossingDirection;
    this.landingPoint = landingPoint;
    setStateFromRequest(BumpCrossingState.FLAT_ABOUT_TO_CROSS);
  }

  /** Abandons any in-progress crossing without re-localizing. */
  public void cancelCrossing() {
    setStateFromRequest(BumpCrossingState.FLAT_NOT_CROSSING);
  }

  @Override
  protected void collectInputs() {
    // Level-referenced: the Pigeon is mounted inverted, so raw roll reads ~180 with the robot
    // flat. Feeding that in flips body-up and inverts the sign of the whole measurement, which
    // would have reported a climb as a descent and never advanced the crossing.
    directionalTilt =
        calculateDirectionalTilt(
            imu.getLevelPitch(), imu.getLevelRoll(), imu.getRobotHeading(), crossingDirection);

    boolean flatNow = Math.abs(directionalTilt) < FLAT_THRESHOLD_DEGREES;
    isFlat = flatDebouncer.calculate(flatNow);
    isFlatFallbackDebounced = flatFallbackDebouncer.calculate(flatNow);
  }

  @Override
  protected BumpCrossingState getNextState(BumpCrossingState currentState) {
    // Fallback: we thought we were climbing but have been flat for a while, so the crossing either
    // already finished or never happened. Finish rather than stay stuck overriding the drive.
    if (currentState == BumpCrossingState.CROSSING_UPHILL && isFlatFallbackDebounced) {
      return finishCrossing("flat fallback", true);
    }

    return switch (currentState) {
      case FLAT_ABOUT_TO_CROSS -> {
        if (directionalTilt > UPHILL_THRESHOLD_DEGREES) {
          yield BumpCrossingState.CROSSING_UPHILL;
        }
        yield timeout(APPROACH_TIMEOUT_SECONDS)
            ? finishCrossing("timeout waiting to climb", false)
            : currentState;
      }
      case CROSSING_UPHILL -> {
        if (directionalTilt < DOWNHILL_THRESHOLD_DEGREES) {
          yield BumpCrossingState.CROSSING_DOWNHILL;
        }
        yield timeout(ON_BUMP_TIMEOUT_SECONDS)
            ? finishCrossing("timeout climbing", false)
            : currentState;
      }
      case CROSSING_DOWNHILL -> {
        if (isFlat) {
          yield finishCrossing("landed", true);
        }
        yield timeout(ON_BUMP_TIMEOUT_SECONDS)
            ? finishCrossing("timeout descending", false)
            : currentState;
      }
      case FLAT_NOT_CROSSING -> currentState;
    };
  }

  /**
   * @param observedFullCrossing whether the tilt sequence actually confirmed a crossing. The
   *     landing pose reset is only applied when it did: a timeout means the bump was never found
   *     or the tilt readings stopped making sense, and teleporting the estimate to a landing point
   *     the robot never reached turns a missed override into a wrong pose. 581 resets on timeout
   *     too, but they have a verified tilt sign and we do not yet.
   */
  private BumpCrossingState finishCrossing(String reason, boolean observedFullCrossing) {
    boolean resetPose = observedFullCrossing && landingPoint.isPresent();
    if (resetPose) {
      landingPoint.ifPresent(poseResetConsumer);
    }

    completedCrossings++;
    SmartDashboard.putString("BumpCrossing/LastFinishReason", reason);
    SmartDashboard.putBoolean("BumpCrossing/LastFinishResetPose", resetPose);
    SmartDashboard.putNumber("BumpCrossing/CompletedCrossings", completedCrossings);

    return BumpCrossingState.FLAT_NOT_CROSSING;
  }

  /**
   * Replaces the along-crossing component of a drive command with a fixed speed while a crossing is
   * in progress. The perpendicular and angular components are preserved, so the path follower still
   * keeps the robot centered on the bump and holds its heading.
   *
   * @param robotRelativeSpeeds the follower's output, robot relative (what BLine emits)
   * @param heading current robot heading, for the frame conversion
   */
  public ChassisVelocities applyCrossingOverride(
      ChassisVelocities robotRelativeSpeeds, Rotation2d heading) {
    if (!getState().isCrossing()) {
      return robotRelativeSpeeds;
    }

    var fieldSpeeds = robotRelativeSpeeds.toFieldRelative(heading);
    double cos = crossingDirection.getCos();
    double sin = crossingDirection.getSin();

    // Component of the follower's command perpendicular to the crossing direction, kept as-is.
    double perpendicular = (-sin * fieldSpeeds.vx) + (cos * fieldSpeeds.vy);

    var overridden =
        new ChassisVelocities(
            (CROSSING_LINEAR_VELOCITY * cos) - (perpendicular * sin),
            (CROSSING_LINEAR_VELOCITY * sin) + (perpendicular * cos),
            fieldSpeeds.omega);

    return overridden.toRobotRelative(heading);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    SmartDashboard.putString("BumpCrossing/State", getState().toString());
    SmartDashboard.putNumber("BumpCrossing/DirectionalTiltDeg", directionalTilt);
    SmartDashboard.putNumber("BumpCrossing/CrossingDirectionDeg", crossingDirection.getDegrees());
    SmartDashboard.putBoolean("BumpCrossing/IsFlat", isFlat);
  }
}
