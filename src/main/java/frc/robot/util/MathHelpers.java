package frc.robot.util;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class MathHelpers {
  private static final Translation2d FIELD_CENTER = new Translation2d(17.55 / 2.0, 8.05 / 2.0);
  private static final double EPSILON = Math.ulp(1.0);

  /**
   * Returns a value rounded to the specified number of decimal places.
   *
   * @param value The value
   * @param numDigits The number of digits after the decimal point to include
   */
  public static double roundTo(double value, double numDigits) {
    var factor = Math.pow(10, numDigits);

    return Math.round(value * factor * (1 + EPSILON)) / factor;
  }

  public static Translation2d roundTo(Translation2d input, double precision) {
    return new Translation2d(roundTo(input.getX(), precision), roundTo(input.getY(), precision));
  }

  public static double average(double a, double b) {
    return (a + b) / 2.0;
  }

  public static double angleModulus(double angleDegrees) {
    return MathUtil.inputModulus(angleDegrees, -180, 180);
  }

  public static Pose2d poseLookahead(Pose2d current, ChassisVelocities velocity, double lookahead) {
    var x = current.getX() + velocity.vx * lookahead;
    var y = current.getY() + velocity.vy * lookahead;
    var theta =
        current
            .getRotation()
            .plus(Rotation2d.fromRadians(velocity.omega * lookahead));

    return new Pose2d(x, y, theta);
  }

  public static double signedExp(double value, double exponent) {
    return Math.copySign(Math.pow(Math.abs(value), exponent), value);
  }

  public static double signedSqrt(double value) {
    return Math.copySign(Math.sqrt(Math.abs(value)), value);
  }

  /**
   * Perform linear interpolation between two ChassisVelocities.
   *
   * @param startValue The ChassisVelocities to start at.
   * @param endValue The ChassisVelocities to end at.
   * @param t How far between the two ChassisVelocities to interpolate. This is clamped to [0, 1].
   * @return The interpolated ChassisVelocities.
   */
  public static ChassisVelocities interpolate(
      ChassisVelocities startValue, ChassisVelocities endValue, double t) {
    return startValue.plus(endValue.minus(startValue).times(Math.clamp(t, 0, 1)));
  }

  /**
   * Returns the input pose flipped from red to blue (or vice versa).
   *
   * @param input Pose to transform
   */
  public static Pose2d pathflip(Pose2d input) {
    return input.rotateAround(FIELD_CENTER, Rotation2d.k180deg);
  }

  public static Rotation2d vectorDirection(ChassisVelocities vector) {
    return new Translation2d(vector.vx, vector.vy).getAngle();
  }

  /**
   * Returns the value that is closer from the two given values. If they are equal, the first value
   * is returned.
   */
  public static double nearest(double value, double a, double b) {
    return Math.abs(value - a) < Math.abs(value - b) ? a : b;
  }

  /**
   * Returns the value that is farther from the two given values. If they are equal, the first value
   * is returned.
   */
  public static double farthest(double value, double a, double b) {
    return Math.abs(value - a) >= Math.abs(value - b) ? a : b;
  }

  private MathHelpers() {}
}
