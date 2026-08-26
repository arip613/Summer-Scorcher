package frc.robot.AutoMovements;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import frc.robot.fms.FmsSubsystem;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.limelight.LimelightHelpers;

// 20,19,18,27,26,25,24,21,30,29,32,31,17,18,23,22
public class HeadingLock extends StateMachine<HeadingLock.HeadingLockState> {
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private Translation2d redTargetPoint = new Translation2d();
  private Translation2d blueTargetPoint = new Translation2d();
  private double operatorOverrideDeg = 0.0;

  private static final String LIMELIGHT_LEFT = "limelight-left";
  // Only refine with tx once the pose-based aim already agrees within this much.
  // Keeps a bad camera calibration from silently overriding a good pose.
  private static final double TX_SWITCH_DEG = 360;
  private static final int[] RED_TAG_PRIORITY = {10, 5, 2};
  private static final int[] BLUE_TAG_PRIORITY = {26, 21, 18};
  
  private static final String USE_TX_KEY = "HeadingLock/UseTx";
  private boolean useTxCheck = true; //-0.5842

  private static final double HEADING_TOLERANCE_DEG = 2;
  private double lastTargetAngleDeg = 0.0;
  private static final double HEADING_SETTLE_TIME_S = 0;
  private double headingOnTargetStartTime = 0;
  private static final double HEADING_TIMEOUT_S = 1.3;
  private double headingLockStartTime = -1.0;

 
  public enum HeadingLockState {
    DISABLED,
    RED_LOCK,
    BLUE_LOCK;
  }



  public HeadingLock(LocalizationSubsystem localization, SwerveSubsystem swerve) {
    super(SubsystemPriority.SWERVE, HeadingLockState.DISABLED);
    this.localization = localization;
    this.swerve = swerve;
    SmartDashboard.putBoolean(USE_TX_KEY, useTxCheck);
  }

  public void setRedTargetPoint(Translation2d point) {
    this.redTargetPoint = point;
  }

  public void setBlueTargetPoint(Translation2d point) {
    this.blueTargetPoint = point;
  }



  public void setRedTargetPose(Pose2d pose) {
    this.redTargetPoint = pose.getTranslation();
  }

  public void setBlueTargetPose(Pose2d pose) {
    this.blueTargetPoint = pose.getTranslation();
  }

  public Pose2d getRedTargetPose() {
    return new Pose2d(redTargetPoint, Rotation2d.kZero);
  }

  public Pose2d getBlueTargetPose() {
    return new Pose2d(blueTargetPoint, Rotation2d.kZero);
  }

  public Translation2d getRedTargetPoint() {
    return redTargetPoint;
  }

  public Translation2d getBlueTargetPoint() {
    return blueTargetPoint;
  }

  public void setOperatorOverrideDeg(double deg) { this.operatorOverrideDeg = deg; }
  public double getOperatorOverrideDeg() { return operatorOverrideDeg; }

  public void enableForAlliance() {
    if (FmsSubsystem.isRedAlliance()) {
      enableRedLock();
    } else {
      enableBlueLock();
    }
  }

  public void enableRedLock() {
    if (getState() != HeadingLockState.RED_LOCK) {
      headingLockStartTime = org.wpilib.system.Timer.getTimestamp();
    }
    setStateFromRequest(HeadingLockState.RED_LOCK);
  }

  public void enableBlueLock() {
    if (getState() != HeadingLockState.BLUE_LOCK) {
      headingLockStartTime = org.wpilib.system.Timer.getTimestamp();
    }
    setStateFromRequest(HeadingLockState.BLUE_LOCK);
  }

  public void disableLock() {
    headingLockStartTime = -1.0;
    setStateFromRequest(HeadingLockState.DISABLED);
    swerve.normalDriveRequest();
  }

  @Override
  protected HeadingLockState getNextState(HeadingLockState current) { return current; }

  private Pose2d getShooterFieldPose() {
    return localization.getPose().transformBy(
        new Transform2d(FieldPoints.SHOOTER_POSE.getTranslation(), FieldPoints.SHOOTER_POSE.getRotation()));
  }

  @Override
  protected void collectInputs() {
    useTxCheck = SmartDashboard.getBoolean(USE_TX_KEY, useTxCheck);
    var shooterTranslation = getShooterFieldPose().getTranslation();
    SmartDashboard.putNumber("HeadingLock/DistTloRed_m",
        shooterTranslation.getDistance(redTargetPoint));
    SmartDashboard.putNumber("HeadingLock/DistToBlue_m",
        shooterTranslation.getDistance(blueTargetPoint));
    switch (getState()) {
      case RED_LOCK -> faceTargetPoseBased(redTargetPoint);
      case BLUE_LOCK -> faceTargetPoseBased(blueTargetPoint);
      case DISABLED -> {}
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
  }

  private void faceTargetPoseBased(Translation2d targetPoint) {

    var shooterPose = getShooterFieldPose();
    double dx = targetPoint.getX() - shooterPose.getX();
    double dy = targetPoint.getY() - shooterPose.getY();

    double distance = Math.hypot(dx, dy);
    if (distance < 1e-6) {
      return;
    }

    double angleToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));

    double poseAngle = angleToTargetDeg + operatorOverrideDeg;
    double currentHeading = localization.getPose().getRotation().getDegrees();
    double poseError = poseAngle - currentHeading;
    poseError = ((poseError + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;

    Double txDegrees = null;
    if (useTxCheck && Math.abs(poseError) <= TX_SWITCH_DEG) {
      txDegrees = getPriorityTagTx();
    }

    double finalAngle = poseAngle;
    if (txDegrees != null) {
      finalAngle = currentHeading - txDegrees + operatorOverrideDeg;
    }



    SmartDashboard.putNumber("HeadingLock/TargetAngleDeg", finalAngle);
    SmartDashboard.putNumber("HeadingLock/PoseTargetDeg", poseAngle);
    SmartDashboard.putNumber("HeadingLock/PoseErrorDeg", poseError);
    SmartDashboard.putBoolean("HeadingLock/UsingTx", txDegrees != null);
    if (txDegrees != null) {
      SmartDashboard.putNumber("HeadingLock/TxDeg", txDegrees);
    }
    SmartDashboard.putNumber("HeadingLock/OperatorOverrideDeg", operatorOverrideDeg);
    SmartDashboard.putNumber("HeadingLock/DistanceM", distance);

    lastTargetAngleDeg = finalAngle;
    swerve.snapsDriveRequest(finalAngle);
  }

  private Double getPriorityTagTx() {
    int[] priority = FmsSubsystem.isRedAlliance() ? RED_TAG_PRIORITY : BLUE_TAG_PRIORITY;
    for (int tagId : priority) {
      Double tx = getTxForTag(tagId);
      if (tx != null) {
        return tx;
      }
    }
    return null;
  }

  private Double getTxForTag(int tagId) {
    return getTxIfMatchingTag(LIMELIGHT_LEFT, tagId);
  }

  private Double getTxIfMatchingTag(String limelightName, int tagId) {
    if (!LimelightHelpers.getTV(limelightName)) {
      return null;
    }
    int fiducial = (int) LimelightHelpers.getFiducialID(limelightName);
    if (fiducial != tagId) {
      return null;
    }
    return LimelightHelpers.getTX(limelightName);
  }

  public boolean isOnTarget() {
    if (getState() == HeadingLockState.DISABLED) return false;

    double now = org.wpilib.system.Timer.getTimestamp();
    boolean timedOut = false;
    if (headingLockStartTime > 0) {
      double elapsed = now - headingLockStartTime;
      timedOut = elapsed >= HEADING_TIMEOUT_S;
      SmartDashboard.putNumber("HeadingLock/TimeoutElapsedS", elapsed);
      SmartDashboard.putBoolean("HeadingLock/TimedOut", timedOut);
    }

    double currentDeg = localization.getPose().getRotation().getDegrees();
    double error = lastTargetAngleDeg - currentDeg;
    error = ((error + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    boolean onTarget = Math.abs(error) <= HEADING_TOLERANCE_DEG || timedOut;
    SmartDashboard.putBoolean("HeadingLock/OnTarget", onTarget);
    SmartDashboard.putNumber("HeadingLock/HeadingErrorDeg", error);

    if (onTarget) {
      if (headingOnTargetStartTime < 0) {
        headingOnTargetStartTime = now;
      }
    } else {
      headingOnTargetStartTime = -1.0;
    }

    return onTarget;
  }

  public boolean isSettled() {
    boolean onTarget = isOnTarget();
    if (!onTarget || headingOnTargetStartTime < 0) {
      SmartDashboard.putBoolean("HeadingLock/Settled", false);
      return false;
    }
    double elapsed = org.wpilib.system.Timer.getTimestamp() - headingOnTargetStartTime;
    boolean settled = elapsed >= HEADING_SETTLE_TIME_S;
    SmartDashboard.putBoolean("HeadingLock/Settled", settled);
    SmartDashboard.putNumber("HeadingLock/SettleElapsedS", elapsed);
    return settled;
  }

}
