package frc.robot.vision.limelight;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.util.Units;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.config.FeatureFlags;
import frc.robot.vision.results.OptionalTagResult;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.CameraHealth;


public class Limelight extends StateMachine<LimelightState> {
  private static final double IS_OFFLINE_TIMEOUT = 3.0;
  private static final double USE_MT1_DISTANCE_THRESHOLD = Units.inchesToMeters(40.0);
  private static final double CONFIG_REASSERT_PERIOD = 1.0;
  /** MegaTag2 only honors robot_orientation_set when the camera uses its external IMU input. */
  private static final int IMU_MODE_EXTERNAL = 0;

  private final String limelightTableName;
  @SuppressWarnings("unused")
  private final String name;
  private final LimelightModel limelightModel;

  private final Timer limelightTimer = new Timer();
  private final Timer configTimer = new Timer();

  /**
   * How much less to trust a one-tag solve. Chosen so that at the 0.12 acceptance gate a single tag
   * is accepted inside roughly 2.5m and rejected beyond it.
   */
  private static final double SINGLE_TAG_STD_DEV_PENALTY_TELEOP = 4.0;

  /**
   * Softer penalty in auto. Auto has no driver to notice a bad pose and correct for it, and it
   * needs a usable estimate more than it needs a perfect one -- match AZGLE4_Q38 ran its whole auto
   * with vision over 200s stale, which also made the bump crossing refuse to arm.
   *
   * <p>38% of frames in that match were single-tag, and at 4x nearly all of them failed the 0.12
   * gate. 2x accepts a single tag out to about 4.4m instead of 2.5m, which covers the range auto
   * actually shoots from, while still trusting it half as much as a multi-tag solve.
   */
  private static final double SINGLE_TAG_STD_DEV_PENALTY_AUTO = 2.0;
  private CameraHealth cameraHealth = CameraHealth.NO_TARGETS;
  private double limelightHeartbeat = -1;
  /** Pipeline the camera was running before we ever corrected it. -1 until first observed. */
  private int firstObservedPipeline = -1;

  private double lastTimestamp = 0.0;

  private double angularVelocity = 0.0;

  private final int[] closestScoringReefTag = {0};
  private OptionalTagResult tagResult = new OptionalTagResult();

  public Limelight(String name, LimelightState initialState, LimelightModel limelightModel) {
    super(SubsystemPriority.VISION, initialState);
    this.limelightTableName = "limelight-" + name;
    this.name = name;
    this.limelightModel = limelightModel;
    limelightTimer.start();
    configTimer.start();
  }

  @Override
  protected void collectInputs() {
    if (configTimer.hasElapsed(CONFIG_REASSERT_PERIOD)) {
      configTimer.reset();
      applyCameraConfig();
    }
  }

  /**
   * Pushes the settings this class assumes onto the camera.
   *
   * <p>{@link LimelightState} has always carried a {@code pipelineIndex}, but nothing ever sent it,
   * so the camera ran whatever pipeline was saved in its own config. If that pipeline is not an
   * AprilTag pipeline, the web UI still shows tag detections while {@code botpose_orb_wpiblue} stays
   * empty -- which reads downstream as "no camera sees anything".
   */
  private void applyCameraConfig() {
    int wantedPipeline = getState().pipelineIndex;
    int actualPipeline = (int) LimelightHelpers.getCurrentPipelineIndex(limelightTableName);
    if (firstObservedPipeline == -1) {
      firstObservedPipeline = actualPipeline;
    }
    SmartDashboard.putNumber(debugKey("PipelineWanted"), wantedPipeline);
    SmartDashboard.putNumber(debugKey("PipelineActual"), actualPipeline);
    SmartDashboard.putNumber(debugKey("PipelineAtBoot"), firstObservedPipeline);
    if (actualPipeline != wantedPipeline) {
      LimelightHelpers.setPipelineIndex(limelightTableName, wantedPipeline);
    }

    // LL4/LL3G have an onboard IMU and can be left in an internal-IMU mode, in which case the yaw
    // pushed by sendImuData is ignored and MegaTag2 produces nothing. This is write-only (there is
    // no readback topic), so just reassert it.
    if (limelightModel == LimelightModel.FOUR || limelightModel == LimelightModel.THREEG) {
      LimelightHelpers.SetIMUMode(limelightTableName, IMU_MODE_EXTERNAL);
    }
  }

  public void sendImuData(
      double robotHeading,
      double angularVelocity,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    LimelightHelpers.SetRobotOrientation(
        limelightTableName, robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    this.angularVelocity = angularVelocity;
  }

  public void setState(LimelightState state) {
    setStateFromRequest(state);
  }

  /**
   * Flushes the last {@code seconds} of the camera's rewind buffer to a .rwnd file on the camera.
   *
   * <p>Rewind records continuously on its own; nothing reaches disk until a capture is triggered,
   * which is why this can be fired after the fact with no cost during the match. Limelight 4 only,
   * and the buffer tops out at 165 seconds.
   *
   * <p>Written as a raw NetworkTables key because the vendored LimelightHelpers in this repo
   * predates setRewindEnabled/triggerRewindCapture.
   */
  public void triggerRewindCapture(double seconds) {
    if (limelightModel != LimelightModel.FOUR) {
      return;
    }
    LimelightHelpers.setLimelightNTDouble(limelightTableName, "capture_rewind", seconds);
    SmartDashboard.putNumber(debugKey("RewindCaptureSeconds"), seconds);
  }


  /**
   * Confidence penalty for a solve built from very few tags.
   *
   * <p>Multiplies the standard deviations, so larger means trusted less. Two tags already fix the
   * pose properly, so only the one-tag case is penalised.
   */
  private static double singleTagPenalty(int tagCount) {
    if (tagCount > 1) {
      return 1.0;
    }
    return org.wpilib.driverstation.RobotState.isAutonomous()
        ? SINGLE_TAG_STD_DEV_PENALTY_AUTO
        : SINGLE_TAG_STD_DEV_PENALTY_TELEOP;
  }

  /** Marks the result empty, records why, and publishes it for debugging. */
  private OptionalTagResult reject(String reason) {
    var empty = tagResult.empty();
    updateHealth(empty);
    SmartDashboard.putString(debugKey("RejectReason"), reason);
    SmartDashboard.putBoolean(debugKey("Accepted"), false);
    return empty;
  }

  private String debugKey(String suffix) {
    return "Vision/" + limelightTableName + "/" + suffix;
  }

  public OptionalTagResult getTagResult() {
    boolean tv = LimelightHelpers.getTV(limelightTableName);
    SmartDashboard.putBoolean(debugKey("TV"), tv);
    SmartDashboard.putNumber(debugKey("RawTV"), tv ? 1 : 0);
    SmartDashboard.putNumber(debugKey("TX"), LimelightHelpers.getTX(limelightTableName));
    // Publish the arrays exactly as received from NetworkTables, before any validity filters. This
    // distinguishes a SystemCore/NT connection problem from a localization rejection.
    double[] rawMt1Pose =
        LimelightHelpers.getLimelightNTDoubleArray(limelightTableName, "botpose_wpiblue");
    double[] rawMt2Pose =
        LimelightHelpers.getLimelightNTDoubleArray(limelightTableName, "botpose_orb_wpiblue");
    boolean mt1Publishing = rawMt1Pose.length >= 6;
    boolean mt2Publishing = rawMt2Pose.length >= 6;
    SmartDashboard.putNumberArray(debugKey("RawBotposeWpiBlue"), rawMt1Pose);
    SmartDashboard.putNumberArray(debugKey("RawBotposeOrbWpiBlue"), rawMt2Pose);
    SmartDashboard.putBoolean(debugKey("MT1Publishing"), mt1Publishing);
    SmartDashboard.putBoolean(debugKey("MT2Publishing"), mt2Publishing);
    SmartDashboard.putNumber(
        debugKey("Heartbeat"),
        LimelightHelpers.getLimelightNTDouble(limelightTableName, "hb"));

    // If this is 0, nothing is publishing to this table name at all -- the camera's hostname in
    // its web UI does not match "limelight-left"/"limelight-right", or it is off the network.
    int tableKeyCount = LimelightHelpers.getLimelightNTTable(limelightTableName).getKeys().size();
    SmartDashboard.putNumber(debugKey("TableKeyCount"), tableKeyCount);
    if (tableKeyCount == 0) {
      return reject("NT table '" + limelightTableName + "' is empty - name mismatch?");
    }

    var mT2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightTableName);
    if (mT2Estimate == null) {
      if (mt1Publishing) {
        return reject("MT1 present, MT2 absent - check robot_orientation_set and IMU mode");
      }
      if (tv) {
        return reject("TV true but no botpose - check field map and camera pose");
      }
      return reject("TV false - no valid AprilTag target");
    }

    SmartDashboard.putNumber(debugKey("TagCount"), mT2Estimate.tagCount);
    SmartDashboard.putNumber(debugKey("AvgTagDist"), mT2Estimate.avgTagDist);
    SmartDashboard.putNumber(debugKey("Timestamp"), mT2Estimate.timestampSeconds);
    SmartDashboard.putNumber(
        debugKey("TimestampAgeS"), org.wpilib.system.Timer.getTimestamp() - mT2Estimate.timestampSeconds);

    if (Math.abs(angularVelocity) > 360) {
      return reject("spinning too fast: " + Math.round(angularVelocity) + " deg/s");
    }

    if (mT2Estimate.tagCount == 0) {
      // MT2 needs robot_orientation_set to be fresh; a stale/absent yaw also lands here.
      return reject("MT2 tagCount 0");
    }

    // Ambiguity is deliberately NOT a rejection criterion here. MegaTag2 resolves the single-tag
    // pose using the yaw pushed via SetRobotOrientation, so it does not suffer the two-solution
    // problem that rawFiducials.ambiguity measures -- that value comes from the raw MT1-style
    // solve, where a tag viewed near head-on is always ambiguous. Live data had the right camera
    // reporting 0.94 on a clean tag-10 sighting at 3.2 m and every frame was being discarded.
    // Published for diagnostics only.
    //
    // Null-checked because LimelightHelpers allocates this array to tagCount but leaves it full of
    // nulls whenever the botpose array length != 11 + 7*tagCount.
    if (mT2Estimate.rawFiducials.length >= 1 && mT2Estimate.rawFiducials[0] != null) {
      SmartDashboard.putNumber(debugKey("Ambiguity"), mT2Estimate.rawFiducials[0].ambiguity);
    }

    if (FeatureFlags.VISION_STALE_DATA_CHECK.getAsBoolean()) {
      var newTimestamp = mT2Estimate.timestampSeconds;
      if (newTimestamp == lastTimestamp) {
        return reject("stale timestamp");
      }
      lastTimestamp = newTimestamp;
    }

    var mt2Pose = mT2Estimate.pose;
    if (mt2Pose.getX() == 0.0 && mt2Pose.getY() == 0.0) {
      return reject("pose is 0,0");
    }

    var devs = VecBuilder.fill(0.01, 0.01, Double.MAX_VALUE);
    if (FeatureFlags.MT_VISION_METHOD.getAsBoolean()) {
      var distance = mT2Estimate.avgTagDist;

      var xyDev = 0.01 * Math.pow(distance, 1.2);
      var thetaDev = 0.03 * Math.pow(distance, 1.2);

      // Scale by how many tags produced the solve. The model above is distance-only, so a
      // single-tag solve was handed to the estimator with exactly the same confidence as a
      // four-tag one -- 0.06 at 4.5m either way. A lone tag has far weaker geometry and is the
      // solve most likely to be wrong.
      //
      // In match AZGLE4_Q29, aiming at the goal with tags around 4.5m, TagCount oscillated
      // 3,4,3,2,1,0,1,0,1,0 and every one-tag frame was fused at full weight. The estimate tore by
      // metres and the snap chased it into a spin.
      //
      // With the penalty a one-tag solve at 4.5m lands at 0.24, past the 0.12 acceptance gate in
      // LocalizationSubsystem, so it is dropped -- while the same solve inside about 2.5m still
      // passes. That is the intent: a close single tag is usable, a distant one is not.
      xyDev *= singleTagPenalty(mT2Estimate.tagCount);
      thetaDev *= singleTagPenalty(mT2Estimate.tagCount);

      boolean isTeleop = org.wpilib.driverstation.RobotState.isTeleop();
      devs = VecBuilder.fill(xyDev, xyDev, isTeleop ? thetaDev : Double.MAX_VALUE);

      if (isTeleop && distance <= USE_MT1_DISTANCE_THRESHOLD) {
        var mT1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightTableName);
        if (mT1Result != null
            && mT1Result.tagCount != 0
            && mT1Result.pose.getRotation().getDegrees() != 0.0) {
          mt2Pose = new Pose2d(mT2Estimate.pose.getTranslation(), mT1Result.pose.getRotation());
        }
      }
    }

    tagResult = tagResult.update(mt2Pose, mT2Estimate.timestampSeconds, devs, mT2Estimate.tagCount);
    updateHealth(tagResult);
    SmartDashboard.putString(debugKey("RejectReason"), "accepted");
    SmartDashboard.putBoolean(debugKey("Accepted"), true);
    SmartDashboard.putNumber(debugKey("XyStdDev"), devs.get(0, 0));
    SmartDashboard.putNumber(debugKey("StdDevTagPenalty"), singleTagPenalty(mT2Estimate.tagCount));
    SmartDashboard.putNumberArray(
        debugKey("Pose"), new double[] {mt2Pose.getX(), mt2Pose.getY(), mt2Pose.getRotation().getDegrees()});
    return tagResult;
  }

  public void setClosestScoringReefTag(int tagID) {
    closestScoringReefTag[0] = tagID;
  }

  private void updateHealth(OptionalTagResult result) {
    var newHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightTableName, "hb");
    if (limelightHeartbeat != newHeartbeat) {
      limelightTimer.reset();
      limelightTimer.start();
    }
    limelightHeartbeat = newHeartbeat;

    if (limelightTimer.hasElapsed(IS_OFFLINE_TIMEOUT) && RobotBase.isReal()) {
      cameraHealth = CameraHealth.OFFLINE;
      return;
    }

    if (result.isPresent()) {
      cameraHealth = CameraHealth.GOOD;
    } else {
      cameraHealth = CameraHealth.NO_TARGETS;
    }
  }

  public CameraHealth getCameraHealth() {
    return cameraHealth;
  }

  public boolean isOnlineForTags() {
    return switch (getState()) {
      case TAGS, OFF -> getCameraHealth() != CameraHealth.OFFLINE;
      default -> false;
    };
  }

}
