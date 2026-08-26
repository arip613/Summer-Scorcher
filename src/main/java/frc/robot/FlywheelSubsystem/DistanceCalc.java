package frc.robot.FlywheelSubsystem;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import frc.robot.AutoMovements.FieldPoints;
import frc.robot.AutoMovements.HeadingLock;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;


public class DistanceCalc extends LifecycleSubsystem {
  private final LocalizationSubsystem localization;
  private final HeadingLock headingLock;

  public DistanceCalc(LocalizationSubsystem localization, HeadingLock headingLock) {
    super(SubsystemPriority.LOCALIZATION);
    this.localization = localization;
    this.headingLock = headingLock;
  }


  /** Returns the shooter's field-relative pose (robot pose + shooter offset). */
  public Pose2d getShooterFieldPose() {
    return localization.getPose().transformBy(
        new Transform2d(FieldPoints.SHOOTER_POSE.getTranslation(), FieldPoints.SHOOTER_POSE.getRotation()));
  }

  public double getDistanceToAllianceTargetMeters() {
    Pose2d current = getShooterFieldPose();
    Pose2d target = FmsSubsystem.isRedAlliance() ? headingLock.getRedTargetPose() : headingLock.getBlueTargetPose();
    return current.getTranslation().getDistance(target.getTranslation());
  }


  public double getRobotVelocityTowardTargetMetersPerSec() {
    Pose2d current = getShooterFieldPose();
    Pose2d target = FmsSubsystem.isRedAlliance() ? headingLock.getRedTargetPose() : headingLock.getBlueTargetPose();

    double dx = target.getX() - current.getX();
    double dy = target.getY() - current.getY();
    double norm = Math.hypot(dx, dy);
    if (norm < 1e-6) {
      return 0.0;
    }

    ChassisVelocities fieldSpeeds = localization.getFieldRelativeSpeeds();
    double vx = fieldSpeeds.vx;
    double vy = fieldSpeeds.vy;

    double ux = dx / norm;
    double uy = dy / norm;

    return vx * ux + vy * uy;
  }


  public double getRobotLateralVelocityMetersPerSec() {
    Pose2d current = getShooterFieldPose();
    Pose2d target = FmsSubsystem.isRedAlliance() ? headingLock.getRedTargetPose() : headingLock.getBlueTargetPose();

    double dx = target.getX() - current.getX();
    double dy = target.getY() - current.getY();
    double norm = Math.hypot(dx, dy);
    if (norm < 1e-6) {
      return 0.0;
    }

    ChassisVelocities fieldSpeeds = localization.getFieldRelativeSpeeds();
    double vx = fieldSpeeds.vx;
    double vy = fieldSpeeds.vy;

    double ux = dx / norm;
    double uy = dy / norm;
    double px = -uy;
    double py = ux;

    return vx * px + vy * py;
  }


  public double getRobotAngularVelocityRadPerSec() {
    return localization.getFieldRelativeSpeeds().omega;
  }

// look ahead exposure
  public org.wpilib.math.geometry.Pose2d getRobotPose() {
    return localization.getPose();
  }

  public double getFieldVelocityX() {
    return localization.getFieldRelativeSpeeds().vx;
  }

  public double getFieldVelocityY() {
    return localization.getFieldRelativeSpeeds().vy;
  }

  public org.wpilib.math.geometry.Pose2d getTargetPose() {
    return FmsSubsystem.isRedAlliance() ? headingLock.getRedTargetPose() : headingLock.getBlueTargetPose();
  }

  // Backwards-compatible accessors used by other subsystems
  public org.wpilib.math.geometry.Pose2d getEstimatedPose() {
    return localization.getPose();
  }

  public org.wpilib.math.kinematics.ChassisVelocities getRobotRelativeVelocity() {
    var field = localization.getFieldRelativeSpeeds();
    var rot = localization.getPose().getRotation();
    double vx = field.vx * rot.getCos() + field.vy * rot.getSin();
    double vy = -field.vx * rot.getSin() + field.vy * rot.getCos();
    return new org.wpilib.math.kinematics.ChassisVelocities(vx, vy, field.omega);
  }

  public org.wpilib.math.geometry.Translation2d getAllianceTargetTranslation() {
    return getTargetPose().getTranslation();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
  }

}
