package frc.robot.localization;

import org.wpilib.system.Timer;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.config.FeatureFlags;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.MathHelpers;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.limelight.LimelightHelpers;
import frc.robot.vision.results.TagResult;
import org.wpilib.framework.RobotBase;

public class LocalizationSubsystem extends StateMachine<LocalizationState> {
  /**
   * Hard rejection threshold for a vision measurement's xy standard deviation.
   *
   * <p>This is a garbage filter, not a quality filter. The standard deviation is already how a
   * measurement is weighted -- the pose estimator moves far less for a 0.24 solve than a 0.06 one --
   * so a tight gate throws away information the filter would have correctly discounted on its own.
   *
   * <p>0.12 was tight enough to be doing the weighting itself. In match AZGLE4_Q49 half the frames
   * in auto were single-tag with tags averaging 4.9m out, which lands them at 0.135 even with auto's
   * softened penalty -- so roughly half of all vision was discarded, the estimator coasted on
   * odometry between accepts, and it had drifted 0.4-0.75m from what the camera could see by the
   * time the robot reached the bump.
   *
   * <p>0.25 admits those, weighted for what they are, while still rejecting a teleop single-tag
   * solve at that range (4x penalty puts it at 0.27) and anything genuinely broken.
   */
  private static final double MAX_VISION_XY_STD_DEV = 0.25;
  private static final int MIN_TAGS_FOR_HEADING = 2;
  private static final double MAX_HEADING_THETA_STD_DEV = 0.05;
  private static final String RIGHT_LIMELIGHT_NAME = "limelight-right";
  private static final double VISION_LOSS_LATCH_TIME_S = 5.0;
  private static final double RECOVERY_SEQUENCE_MAX_GAP_S = 0.25;
  private static final double HARD_RESET_MIN_TRANSLATION_ERROR_M = 0.25;
  /**
   * Reject vision while the robot is tilted more than this far off level. A tag solve assumes the
   * camera sits at its calibrated offset from the robot origin; when the robot pitches or rolls
   * over a bump the camera swings well outside that offset, and the solve comes back confidently
   * wrong rather than merely noisy. Matches the flat threshold BumpCrossingTracker uses.
   */
  private static final double MAX_VISION_TILT_DEGREES = 4.0;

  /**
   * How stale the last accepted vision measurement may be before the pose stops being trusted for
   * open-loop moves. Odometry alone drifts slowly, so a second is generous for "vision agreed with
   * me recently"; anything older means nothing has confirmed the estimate.
   */
  private static final double POSE_TRUST_MAX_VISION_AGE_S = 1.0;

  /**
   * How long after a hard pose reset the estimate stays untrusted. A reset means the estimate had
   * already drifted at least HARD_RESET_MIN_TRANSLATION_ERROR_M, and the new one has not been
   * corroborated yet.
   */
  private static final double POSE_TRUST_SETTLE_AFTER_RESET_S = 1.0;


  private final ImuSubsystem imu;
  private final VisionSubsystem vision;
  private final SwerveSubsystem swerve;
  private final DoubleArrayPublisher botposeBluePub;
  private final DoubleArrayPublisher robotPosePub;
  private int visionAcceptedCount = 0;
  private int visionHardResetCount = 0;
  private double lastHardResetTime = Double.NEGATIVE_INFINITY;
  private final VisionRecoveryGate visionRecoveryGate = new VisionRecoveryGate();
  private double lastAcceptedVisionRobotTime = Timer.getTimestamp();
  private double lastRecoveryCandidateRobotTime = -1.0;
  private boolean visionRecoveryPending;
  private Pose2d poseBeforePendingVisionUpdate;

  public LocalizationSubsystem(ImuSubsystem imu, VisionSubsystem vision, SwerveSubsystem swerve) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.imu = imu;
    this.vision = vision;

    var nt = NetworkTableInstance.getDefault();
    botposeBluePub = nt.getTable("limelight").getDoubleArrayTopic("botpose_blue").publish();
    robotPosePub = nt.getTable("Localization").getDoubleArrayTopic("robot_pose").publish();

    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo180",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(180))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo0",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(0))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo90",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(90))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo270",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(270))).ignoringDisable(true));
    }
  }

  public Pose2d getPose() {
    return swerve.getDrivetrainState().Pose;
  }

  public Pose2d getPose(double timestamp) {
    if (RobotBase.isSimulation()) {
      return getPose();
    }
    // Phoenix 6 and WPILib share one time base on SystemCore, so no conversion is needed.
    return swerve.drivetrain.samplePoseAt(timestamp).orElseGet(this::getPose);
  }

  public Pose2d getLookaheadPose(double lookahead) {
    return MathHelpers.poseLookahead(getPose(), swerve.getFieldRelativeSpeeds(), lookahead);
  }

  public ChassisVelocities getFieldRelativeSpeeds() {
    return swerve.getFieldRelativeSpeeds();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    double now = Timer.getTimestamp();
    if (poseBeforePendingVisionUpdate != null) {
      SmartDashboard.putNumber(
          "Localization/VisionDeltaAppliedM",
          poseBeforePendingVisionUpdate.getTranslation().getDistance(getPose().getTranslation()));
      poseBeforePendingVisionUpdate = null;
    }

    if (!visionRecoveryPending
        && now - lastAcceptedVisionRobotTime >= VISION_LOSS_LATCH_TIME_S) {
      visionRecoveryPending = true;
      visionRecoveryGate.reset();
      lastRecoveryCandidateRobotTime = -1.0;
      SmartDashboard.putString("Localization/VisionRecoveryStatus", "vision loss latched");
    }

    var right = vision.getRightTagResult();
    if (right.isEmpty()) {
      // Only the right camera is permitted to update translation. The per-camera RejectReason key
      // explains why it did not produce a usable result.
      SmartDashboard.putString("Localization/VisionStatus", "no usable right-camera pose result");
    }
    TagResult rightRecoveryCandidate = null;
    TagResult rightResult = right.orElse(null);
    if (rightResult != null && ingestTagResult(rightResult, now)) {
      rightRecoveryCandidate = rightResult;
    }

    if (rightRecoveryCandidate != null) {
      lastAcceptedVisionRobotTime = now;
      if (visionRecoveryPending) {
        processRecoveryCandidate(rightRecoveryCandidate, now);
      }
    } else if (visionRecoveryPending
        && visionRecoveryGate.getConsistentFrames() > 0
        && now - lastRecoveryCandidateRobotTime > RECOVERY_SEQUENCE_MAX_GAP_S) {
      visionRecoveryGate.reset();
      SmartDashboard.putNumber("Localization/VisionRecoveryFrames", 0);
      SmartDashboard.putString("Localization/VisionRecoveryStatus", "candidate gap");
    }

    SmartDashboard.putBoolean("Localization/VisionRecoveryPending", visionRecoveryPending);
    SmartDashboard.putNumber("Localization/TiltFromLevelDeg", tiltFromLevelDegrees());
    SmartDashboard.putBoolean("Localization/PoseTrusted", isPoseTrusted());
    SmartDashboard.putNumber(
        "Localization/VisionAgeS", Timer.getTimestamp() - lastAcceptedVisionRobotTime);

    updateHeadingFromRightVision();

    Pose2d pose = getPose();
    SmartDashboard.putNumberArray(
        "Localization/EstimatedPose",
        new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
    botposeBluePub.set(new double[] {
        pose.getX(),
        pose.getY(),
        0.0, 
        0.0, 
        0.0, 
        pose.getRotation().getDegrees()
    });

    robotPosePub.set(new double[] {
        pose.getX(),
        pose.getY(),
        pose.getRotation().getRadians()
    });

  }

  /**
   * Angle between robot-up and field-up, in degrees. Combines pitch and roll into a single tilt
   * magnitude so the gate holds at any heading -- crossing a bump sideways shows up as roll, not
   * pitch, and checking only one axis would miss it.
   */
  private double tiltFromLevelDegrees() {
    // Level-referenced, not raw. The Pigeon is mounted inverted and reads roll ~179 with the robot
    // flat, so using the raw values made a level robot measure as 179 degrees tilted and rejected
    // every vision measurement for a whole match.
    double pitch = Math.toRadians(imu.getLevelPitch());
    double roll = Math.toRadians(imu.getLevelRoll());
    double tilt =
        Math.toDegrees(Math.acos(Math.clamp(Math.cos(pitch) * Math.cos(roll), -1.0, 1.0)));

    // Fold about 90 degrees. If the mount offset is ever wrong again the raw angle lands near 180
    // for a level robot; folding makes that cost accuracy near 90 degrees of tilt instead of
    // silently disabling vision entirely. A robot past 90 degrees has bigger problems than pose.
    return Math.min(tilt, 180.0 - tilt);
  }

  /**
   * Whether the pose estimate is currently good enough to act on open loop.
   *
   * <p>Closed-loop path following degrades gracefully with a bad pose -- it drives to the wrong
   * place but keeps correcting. An open-loop move does not: it commits to a heading and a speed
   * derived from the estimate at one instant and cannot notice it was wrong. So anything that
   * gives up feedback needs to ask first.
   *
   * <p>In match AZGLE4_Q29 the robot hit a teammate on the approach to the bump, the estimate
   * drifted far enough to trigger a hard reset at t=170.3, and the bump crossing was armed from
   * 169.31. It only avoided driving on the discarded pose because the approach timed out first.
   */
  public boolean isPoseTrusted() {
    double now = Timer.getTimestamp();

    return !visionRecoveryPending
        && (now - lastAcceptedVisionRobotTime) <= POSE_TRUST_MAX_VISION_AGE_S
        && (now - lastHardResetTime) > POSE_TRUST_SETTLE_AFTER_RESET_S;
  }

  private void updateHeadingFromRightVision() {
    if (tiltFromLevelDegrees() > MAX_VISION_TILT_DEGREES) {
      SmartDashboard.putString("Localization/VisionStatus", "rejected: robot tilted");
      return;
    }

    // Always blue-origin: the drivetrain pose estimator is blue-origin, so a red-origin
    // botpose would be flipped 180 degrees.
    var mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(RIGHT_LIMELIGHT_NAME);
    if (mt1Estimate == null || mt1Estimate.tagCount < MIN_TAGS_FOR_HEADING) {
      return;
    }

    double targetHeadingDeg = mt1Estimate.pose.getRotation().getDegrees();

    // Fuse heading through the pose estimator instead of hard-resetting the gyro.
    // Use the current XY with large XY std devs so only the heading component
    // meaningfully corrects the estimate.
    Pose2d current = getPose();
    Pose2d headingPose = new Pose2d(
        current.getTranslation(),
        Rotation2d.fromDegrees(targetHeadingDeg));
    var headingStdDevs = org.wpilib.math.linalg.VecBuilder.fill(
        100.0, 100.0, MAX_HEADING_THETA_STD_DEV);
    swerve.drivetrain.addVisionMeasurement(
        headingPose, mt1Estimate.timestampSeconds, headingStdDevs);
  }

  private boolean ingestTagResult(TagResult result, double now) {
    var visionPose = result.pose();

    double tiltDegrees = tiltFromLevelDegrees();
    if (tiltDegrees > MAX_VISION_TILT_DEGREES) {
      SmartDashboard.putString("Localization/VisionStatus", "rejected: robot tilted");
      SmartDashboard.putNumber("Localization/VisionRejectedTiltDeg", tiltDegrees);
      return false;
    }

    if (!isStdDevAcceptable(result.standardDevs())) {
      SmartDashboard.putString("Localization/VisionStatus", "rejected: xy std dev too high");
      return false;
    }

    double timestampAge = now - result.timestamp();
    if (timestampAge < VisionRecoveryGate.MIN_TIMESTAMP_AGE_SEC
        || timestampAge > VisionRecoveryGate.MAX_TIMESTAMP_AGE_SEC) {
      SmartDashboard.putString("Localization/VisionStatus", "rejected: unreasonable timestamp");
      SmartDashboard.putNumber("Localization/VisionTimestampAgeS", timestampAge);
      return false;
    }

    var poseXYOnly = new Pose2d(
        visionPose.getTranslation(),
        swerve.getDrivetrainState().Pose.getRotation());

    // How far the estimator was from the camera before this measurement, and how much of that
    // gap the estimator actually closed. If ErrorBefore is large but DeltaApplied stays ~0, the
    // measurement is being dropped (bad timestamp) or drowned out by setStateStdDevs.
    Pose2d before = getPose();
    double errorBefore = before.getTranslation().getDistance(poseXYOnly.getTranslation());

    swerve.drivetrain.addVisionMeasurement(
        poseXYOnly, result.timestamp(), result.standardDevs());
    poseBeforePendingVisionUpdate = before;

    SmartDashboard.putString("Localization/VisionStatus", "accepted");
    SmartDashboard.putNumber("Localization/VisionAccepted", ++visionAcceptedCount);
    SmartDashboard.putNumber("Localization/VisionErrorBeforeM", errorBefore);
    SmartDashboard.putNumber("Localization/VisionTimestampAgeS", timestampAge);
    return true;
  }

  private void processRecoveryCandidate(TagResult result, double now) {
    double xyStdDev = getXyStdDev(result);
    double timestampAge = now - result.timestamp();
    var evaluation = visionRecoveryGate.evaluate(
        result.pose().getTranslation(),
        result.tagCount(),
        xyStdDev,
        imu.getRobotAngularVelocity(),
        timestampAge);
    lastRecoveryCandidateRobotTime = now;

    SmartDashboard.putNumber(
        "Localization/VisionRecoveryFrames", evaluation.consistentFrames());
    SmartDashboard.putString("Localization/VisionRecoveryStatus", evaluation.status());
    SmartDashboard.putNumber("Localization/VisionRecoveryCandidateStdDevM", xyStdDev);
    SmartDashboard.putNumber("Localization/VisionRecoveryTagCount", result.tagCount());

    if (!evaluation.ready()) {
      return;
    }

    double disagreement = getPose().getTranslation().getDistance(evaluation.averagedTranslation());
    SmartDashboard.putNumber("Localization/VisionRecoveryDisagreementM", disagreement);
    if (disagreement >= HARD_RESET_MIN_TRANSLATION_ERROR_M) {
      resetPoseXYOnly(new Pose2d(
          evaluation.averagedTranslation(), getPose().getRotation()));
      lastHardResetTime = Timer.getTimestamp();
      SmartDashboard.putNumber("Localization/VisionHardResets", ++visionHardResetCount);
      SmartDashboard.putString("Localization/VisionRecoveryStatus", "XY reset applied");
    } else {
      SmartDashboard.putString(
          "Localization/VisionRecoveryStatus", "normal fusion sufficient");
    }

    visionRecoveryPending = false;
    visionRecoveryGate.reset();
    SmartDashboard.putNumber("Localization/VisionRecoveryFrames", 0);
  }

  private static double getXyStdDev(TagResult result) {
    if (result.standardDevs() == null) {
      return Double.POSITIVE_INFINITY;
    }
    return Math.max(
        result.standardDevs().get(0, 0), result.standardDevs().get(1, 0));
  }

  private boolean isStdDevAcceptable(org.wpilib.math.linalg.Vector<org.wpilib.math.numbers.N3> devs) {
    if (devs == null) {
      return false;
    }
    // Only XY is checked: ingestTagResult discards the vision rotation and keeps the
    // estimator's own heading, so the theta std dev is irrelevant here. Checking it
    // rejected every measurement in auto, where theta dev is deliberately MAX_VALUE.
    double xyStd = Math.max(devs.get(0, 0), devs.get(1, 0));
    return xyStd <= MAX_VISION_XY_STD_DEV;
  }

  public void resetGyro(Rotation2d gyroAngle) {
    // Only use CTRE's offset math — do NOT call Pigeon2.setYaw().
    // setYaw is a CAN command that takes 1-2ms to propagate. If resetRotation
    // reads the gyro before setYaw arrives, the offset is computed against the
    // stale value. When setYaw finally propagates, the heading jumps to
    // 2*desired - oldRaw (typically 90° off).
    swerve.resetRotation(gyroAngle);
  }

  public void resetPose(Pose2d estimatedPose) {
    // resetPose sets both position and rotation in one atomic operation.
    // No separate resetRotation needed — it would just be overwritten.
    swerve.resetPose(estimatedPose);
  }

  /**
   * Resets only the translation of the pose estimate, keeping the estimator's own heading. Used by
   * the bump crossing tracker to re-localize on a known landing point, where the gyro heading is
   * still trustworthy but wheel odometry through the bump is not.
   */
  public void resetTranslationOnly(Translation2d translation) {
    swerve.resetPose(new Pose2d(translation, swerve.getDrivetrainState().Pose.getRotation()));
  }

  public void resetPoseXYOnly(Pose2d estimatedPose) {
    swerve.resetPose(
        new Pose2d(estimatedPose.getTranslation(), swerve.getDrivetrainState().Pose.getRotation()));
  }

  public Command getZeroCommand() {
    return Commands.runOnce(
        () -> resetGyro(Rotation2d.fromDegrees((FmsSubsystem.isRedAlliance() ? 180 : 0))));
  }
}
