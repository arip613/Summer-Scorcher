package frc.robot.autos.constraints;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import frc.robot.util.TimestampedChassisSpeeds;

public class AutoConstraintCalculator {
  private static AutoConstraintOptions lastUsedConstraints = new AutoConstraintOptions();

  public static TimestampedChassisSpeeds constrainVelocityGoal(
      TimestampedChassisSpeeds inputSpeeds,
      TimestampedChassisSpeeds previousSpeeds,
      AutoConstraintOptions options,
      double distanceToSegmentEnd) {
    ChassisVelocities constrainedSpeeds = constrainVelocityGoal(inputSpeeds, previousSpeeds, options);

    double newLinearVelocity =
        getAccelerationBasedVelocityConstraint(
            constrainedSpeeds,
            distanceToSegmentEnd,
            options.maxLinearAcceleration(),
            options.maxLinearVelocity());
    constrainedSpeeds =
        constrainLinearVelocity(
            constrainedSpeeds, options.withMaxLinearVelocity(newLinearVelocity));

    return new TimestampedChassisSpeeds(constrainedSpeeds, inputSpeeds.timestampSeconds);
  }

  public static ChassisVelocities constrainVelocityGoal(
      TimestampedChassisSpeeds inputSpeeds,
      TimestampedChassisSpeeds previousSpeeds,
      AutoConstraintOptions options) {
    lastUsedConstraints = options;
    var constrainedSpeeds = inputSpeeds;

    if (options.maxLinearVelocity() != 0) {
      constrainedSpeeds =
          new TimestampedChassisSpeeds(
              constrainLinearVelocity(constrainedSpeeds, options),
              constrainedSpeeds.timestampSeconds);
    }

    if (options.maxAngularVelocity() != 0) {
      constrainedSpeeds =
          new TimestampedChassisSpeeds(
              constrainRotationalVelocity(constrainedSpeeds, options),
              constrainedSpeeds.timestampSeconds);
    }

  // Linear acceleration constraint temporarily disabled in simplified build.

    if (options.maxAngularAcceleration() != 0) {
      constrainedSpeeds =
          new TimestampedChassisSpeeds(
              constrainRotationalAcceleration(constrainedSpeeds, previousSpeeds, options),
              constrainedSpeeds.timestampSeconds);
    }

    return constrainedSpeeds;
  }

  public static AutoConstraintOptions getLastUsedConstraints() {
    return lastUsedConstraints;
  }

  public static ChassisVelocities constrainLinearVelocity(
      ChassisVelocities inputSpeeds, AutoConstraintOptions options) {
    double currentLinearVelocity =
        Math.hypot(inputSpeeds.vx, inputSpeeds.vy);
    // double preserveTheta = Math.atan(inputSpeeds.vy /
    // inputSpeeds.vx);
    if (currentLinearVelocity > options.maxLinearVelocity()) {
      double clampingFactor = options.maxLinearVelocity() / currentLinearVelocity;

      return new ChassisVelocities(
          inputSpeeds.vx * clampingFactor,
          inputSpeeds.vy * clampingFactor,
          inputSpeeds.omega);
    }
    return inputSpeeds;
  }

  private static ChassisVelocities constrainRotationalVelocity(
      ChassisVelocities inputSpeeds, AutoConstraintOptions options) {
    double currentAngularVelocity = inputSpeeds.omega;
    if (currentAngularVelocity > options.maxAngularVelocity()) {
      double clampingFactor = options.maxAngularVelocity() / currentAngularVelocity;
      return new ChassisVelocities(
          inputSpeeds.vx,
          inputSpeeds.vy,
          inputSpeeds.omega * clampingFactor);
    }

    return inputSpeeds;
  }

  @SuppressWarnings("unused")
  private static ChassisVelocities constrainLinearAcceleration(
      TimestampedChassisSpeeds inputSpeeds,
      TimestampedChassisSpeeds previousSpeeds,
      AutoConstraintOptions options) {

    double inputTotalSpeed =
        Math.sqrt(
            Math.pow(inputSpeeds.vx, 2)
                + Math.pow(inputSpeeds.vy, 2));
    double previousTotalSpeed =
        Math.sqrt(
            Math.pow(previousSpeeds.vx, 2)
                + Math.pow(previousSpeeds.vy, 2));
    double unconstrainedLinearAcceleration =
        (inputTotalSpeed - previousTotalSpeed) / inputSpeeds.timestampDifference(previousSpeeds);

    if (unconstrainedLinearAcceleration < 0) {
      return inputSpeeds;
    }

    double deltaVx = inputSpeeds.vx - previousSpeeds.vx;
    double deltaVy = inputSpeeds.vy - previousSpeeds.vy;

    double constrainedLinearAcceleration =
        Math.min(unconstrainedLinearAcceleration, options.maxLinearAcceleration());

    if (unconstrainedLinearAcceleration > options.maxLinearAcceleration()) {
      double constrainedVx =
          previousSpeeds.vx
              + (deltaVx / unconstrainedLinearAcceleration) * constrainedLinearAcceleration;
      double constrainedVy =
          previousSpeeds.vy
              + (deltaVy / unconstrainedLinearAcceleration) * constrainedLinearAcceleration;

      return new ChassisVelocities(constrainedVx, constrainedVy, inputSpeeds.omega);
    }
    return inputSpeeds;
  }

  public static double getAccelerationBasedVelocityConstraint(
      ChassisVelocities currentSpeeds,
      double distanceToSegmentEnd,
      double accelerationLimit,
      double velocityConstraint) {
    double currentVelocity =
        Math.hypot(currentSpeeds.vx, currentSpeeds.vy);
    double decelerationDistance =
        (1.0 * (currentVelocity * currentVelocity)) / (2.0 * accelerationLimit);
    double perfectVelocity =
        Math.sqrt(0.0 - (-1.0 * 2.0 * (accelerationLimit * distanceToSegmentEnd)));

    if (distanceToSegmentEnd > decelerationDistance) {
      return currentVelocity;
    }
    return Math.max(perfectVelocity, 0.05); // Allow you to go 2 inches per second
  }

  public static double getDynamicVelocityConstraint(
      Pose2d currentPose,
      Pose2d endWaypoint,
      ChassisVelocities currentSpeeds,
      double oldVelocityConstraint,
      double accelerationLimit) {
    var distanceToEnd = currentPose.getTranslation().getDistance(endWaypoint.getTranslation());
    var currentVelocity =
        Math.hypot(currentSpeeds.vx, currentSpeeds.vy);

    var timeToTraverse = distanceToEnd / currentVelocity;
    var acceleration = (accelerationLimit - currentVelocity) / timeToTraverse;
    if (Math.abs(acceleration) < accelerationLimit) {
      return oldVelocityConstraint;
    }
    var velocityConstraint = acceleration * timeToTraverse;
    var clampedConstraint = Math.clamp(Math.abs(velocityConstraint), 0.5, 5.0);
    return clampedConstraint;
  }

  private static ChassisVelocities constrainRotationalAcceleration(
      TimestampedChassisSpeeds inputSpeeds,
      TimestampedChassisSpeeds previousSpeeds,
      AutoConstraintOptions options) {

    double currentAngularSpeed = inputSpeeds.omega;
    double previousAngularSpeed = previousSpeeds.omega;

    double currentAngularAcceleration =
        currentAngularSpeed
            - previousAngularSpeed / inputSpeeds.timestampDifference(previousSpeeds);
    if (currentAngularAcceleration > options.maxAngularAcceleration()) {
      double constrainedAngularAcceleration =
          previousAngularSpeed
              + options.maxAngularAcceleration() * inputSpeeds.timestampDifference(previousSpeeds);
      return new ChassisVelocities(
          inputSpeeds.vx,
          inputSpeeds.vy,
          constrainedAngularAcceleration);
    }
    return inputSpeeds;
  }

  private AutoConstraintCalculator() {}
}
