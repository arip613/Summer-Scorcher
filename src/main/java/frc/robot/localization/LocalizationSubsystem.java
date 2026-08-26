package frc.robot.localization;

import org.wpilib.system.Timer;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
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

public class LocalizationSubsystem extends StateMachine<LocalizationState> {
  private static final double MAX_VISION_XY_STD_DEV = 0.12;
  private static final int MIN_TAGS_FOR_HEADING = 2;
  private static final double MAX_HEADING_THETA_STD_DEV = 0.05;
  private static final String RIGHT_LIMELIGHT_NAME = "limelight-right";


  private final ImuSubsystem imu;
  private final VisionSubsystem vision;
  private final SwerveSubsystem swerve;
  private final DoubleArrayPublisher botposeBluePub;
  private final DoubleArrayPublisher robotPosePub;
  private int visionAcceptedCount = 0;
  private int visionHardResetCount = 0;

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

    var left = vision.getLeftTagResult();
    var right = vision.getRightTagResult();
    if (left.isEmpty() && right.isEmpty()) {
      // Neither camera produced a usable result; the per-camera RejectReason keys say why.
      SmartDashboard.putString("Localization/VisionStatus", "no tag result from either camera");
    }
    left.ifPresent(this::ingestTagResult);
    right.ifPresent(this::ingestTagResult);

    updateHeadingFromVision();

    Pose2d pose = getPose();
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

  private void updateHeadingFromVision() {
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

  private void ingestTagResult(TagResult result) {
    var visionPose = result.pose();

    if (!isStdDevAcceptable(result.standardDevs())) {
      SmartDashboard.putString("Localization/VisionStatus", "rejected: xy std dev too high");
      return;
    }

    if (!vision.seenTagRecentlyForReset() && FeatureFlags.MT_VISION_METHOD.getAsBoolean()) {
      resetPoseXYOnly(visionPose);
      SmartDashboard.putNumber("Localization/VisionHardResets", ++visionHardResetCount);
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

    Pose2d after = swerve.drivetrain.getState().Pose;
    double deltaApplied = before.getTranslation().getDistance(after.getTranslation());

    SmartDashboard.putString("Localization/VisionStatus", "accepted");
    SmartDashboard.putNumber("Localization/VisionAccepted", ++visionAcceptedCount);
    SmartDashboard.putNumber("Localization/VisionErrorBeforeM", errorBefore);
    SmartDashboard.putNumber("Localization/VisionDeltaAppliedM", deltaApplied);
    SmartDashboard.putNumber(
        "Localization/VisionTimestampAgeS", Timer.getTimestamp() - result.timestamp());
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
    swerve.drivetrain.resetRotation(gyroAngle);
  }

  public void resetPose(Pose2d estimatedPose) {
    // resetPose sets both position and rotation in one atomic operation.
    // No separate resetRotation needed — it would just be overwritten.
    swerve.drivetrain.resetPose(estimatedPose);
  }

  public void resetPoseXYOnly(Pose2d estimatedPose) {
    swerve.drivetrain.resetPose(
        new Pose2d(estimatedPose.getTranslation(), swerve.getDrivetrainState().Pose.getRotation()));
  }

  public Command getZeroCommand() {
    return Commands.runOnce(
        () -> resetGyro(Rotation2d.fromDegrees((FmsSubsystem.isRedAlliance() ? 180 : 0))));
  }
}
