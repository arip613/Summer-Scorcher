package frc.robot.util;

import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

public class TimestampedChassisSpeeds extends ChassisVelocities {
  public final double timestampSeconds;

  public TimestampedChassisSpeeds(double vx, double vy, double omega, double timestampSeconds) {
    super(vx, vy, omega);
    this.timestampSeconds = timestampSeconds;
  }

  public TimestampedChassisSpeeds(double vx, double vy, double omega) {
    this(vx, vy, omega, Timer.getTimestamp());
  }

  public TimestampedChassisSpeeds(ChassisVelocities speeds, double timestampSeconds) {
    this(
        speeds.vx,
        speeds.vy,
        speeds.omega,
        timestampSeconds);
  }

  public TimestampedChassisSpeeds(ChassisVelocities speeds) {
    this(speeds.vx, speeds.vy, speeds.omega);
  }

  public TimestampedChassisSpeeds(TimestampedChassisSpeeds speeds) {
    this(
        speeds.vx,
        speeds.vy,
        speeds.omega,
        speeds.timestampSeconds);
  }

  public TimestampedChassisSpeeds(double timestampSeconds) {
    this(0, 0, 0, timestampSeconds);
  }

  public TimestampedChassisSpeeds() {
    this(Timer.getTimestamp());
  }

  public double timestampDifference(TimestampedChassisSpeeds other) {
    return timestampSeconds - other.timestampSeconds;
  }
}
