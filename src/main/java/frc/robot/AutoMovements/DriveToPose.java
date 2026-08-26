package frc.robot.AutoMovements;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import java.util.function.Supplier;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;

public class DriveToPose extends Command {
  private static final double MAX_SPEED = 6;
  private static final double MAX_ACCEL = 3.5;
  private static final double DT = 0.02;
  private static final double DRIVE_TOLERANCE = 0.5; 
  private static final double THETA_TOLERANCE = 5.0; 

  private static final double PHASE_TRANSITION_Y_TOLERANCE = 0.4; 
  private static final double PHASE_TRANSITION_THETA_TOLERANCE = 15.0; 

  private enum Phase { Y_AND_HEADING, ALL }

  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final Supplier<Pose2d> targetSupplier;
  private final PIDController xController = new PIDController(3, 0.0, 0.0);
  private final PIDController yController = new PIDController(3, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(4.0, 0.0, 0.1);

  private final boolean simultaneous;
  private final double maxSpeed;

  private Pose2d targetPose;
  private Phase phase;
  private double prevVx = 0.0;
  private double prevVy = 0.0;

  public DriveToPose(SwerveSubsystem swerve, LocalizationSubsystem localization, Supplier<Pose2d> target) {
    this(swerve, localization, target, false, MAX_SPEED);
  }

  public DriveToPose(SwerveSubsystem swerve, LocalizationSubsystem localization, Supplier<Pose2d> target, boolean simultaneous) {
    this(swerve, localization, target, simultaneous, MAX_SPEED);
  }


  public DriveToPose(SwerveSubsystem swerve, LocalizationSubsystem localization, Supplier<Pose2d> target, boolean simultaneous, double maxSpeed) {
    this.swerve = swerve;
    this.localization = localization;
    this.targetSupplier = target;
    this.simultaneous = simultaneous;
    this.maxSpeed = (maxSpeed > 0) ? maxSpeed : MAX_SPEED;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    targetPose = targetSupplier.get();
    xController.reset();
    yController.reset();
    thetaController.reset();
    phase = simultaneous ? Phase.ALL : Phase.Y_AND_HEADING;
    prevVx = 0.0;
    prevVy = 0.0;
    SmartDashboard.putString("DriveToPose/Target",
        String.format("(%.2f, %.2f, %.1f deg)", targetPose.getX(), targetPose.getY(),
            targetPose.getRotation().getDegrees()));
  }

  @Override
  public void execute() {
    Pose2d current = localization.getPose();
    double ySpeed = yController.calculate(current.getY(), targetPose.getY());
    double thetaSpeed = thetaController.calculate(
        current.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaSpeed = Math.clamp(thetaSpeed, -Math.PI * 2, Math.PI * 2);
    double xSpeed = 0.0;
    double yError = Math.abs(current.getY() - targetPose.getY());
    double thetaError = Math.abs(current.getRotation().minus(targetPose.getRotation()).getDegrees());
    if (phase == Phase.Y_AND_HEADING) {
      if (yError < PHASE_TRANSITION_Y_TOLERANCE && thetaError < PHASE_TRANSITION_THETA_TOLERANCE) {
        phase = Phase.ALL;
      }
    }
    if (phase == Phase.ALL) {
      xSpeed = xController.calculate(current.getX(), targetPose.getX());
    }
    Translation2d linearVelocity = new Translation2d(xSpeed, ySpeed);
    double magnitude = linearVelocity.getNorm();
    if (magnitude > maxSpeed) {
      linearVelocity = linearVelocity.times(maxSpeed / magnitude);
    }
    double maxDelta = MAX_ACCEL * DT;
    double dvx = linearVelocity.getX() - prevVx;
    double dvy = linearVelocity.getY() - prevVy;
    double deltaMag = Math.hypot(dvx, dvy);
    if (deltaMag > maxDelta) {
      double scale = maxDelta / deltaMag;
      dvx *= scale;
      dvy *= scale;
    }
    double outVx = prevVx + dvx;
    double outVy = prevVy + dvy;
    prevVx = outVx;
    prevVy = outVy;
    swerve.setFieldRelativeAutoSpeeds(new ChassisVelocities(outVx, outVy, thetaSpeed));
    double driveError = current.getTranslation().getDistance(targetPose.getTranslation());
    SmartDashboard.putNumber("DriveToPose/DriveError", driveError);
    SmartDashboard.putNumber("DriveToPose/YError", yError);
    SmartDashboard.putNumber("DriveToPose/ThetaErrorDeg", thetaError);
    SmartDashboard.putString("DriveToPose/Phase", phase.name());
    SmartDashboard.putBoolean("DriveToPose/AtGoal", isAtGoal());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setFieldRelativeAutoSpeeds(new ChassisVelocities(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return isAtGoal();
  }

  private boolean isAtGoal() {
    Pose2d current = localization.getPose();
    double driveError = current.getTranslation().getDistance(targetPose.getTranslation());
    double thetaError = Math.abs(current.getRotation().minus(targetPose.getRotation()).getDegrees());
    return driveError < DRIVE_TOLERANCE && thetaError < THETA_TOLERANCE;
  }

  public boolean isInAllPhase() {
    return phase == Phase.ALL;
  }

  public double getDistanceToTarget() {
    if (targetPose == null) return Double.MAX_VALUE;
    return localization.getPose().getTranslation().getDistance(targetPose.getTranslation());
  }
}