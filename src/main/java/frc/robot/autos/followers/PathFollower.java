package frc.robot.autos.followers;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Generates swerve setpoints using robot pose and target pose. */
public interface PathFollower {
  /**
   * Given the current pose of the robot and the target pose, calculate the velocity the robot
   * should drive at to get there.
   *
   * @param currentPose The current pose of the robot.
   * @param targetPose The target pose to drive to.
   * @return The desired robot velocity to drive at.
   */
  public ChassisVelocities calculateSpeeds(Pose2d currentPose, Pose2d targetPose);
}
