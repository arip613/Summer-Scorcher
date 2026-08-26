package frc.robot.autos.followers;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class PidPathFollower implements PathFollower {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  public PidPathFollower(
      PIDController xController, PIDController yController, PIDController thetaController) {
    this.xController = xController;
    this.yController = yController;
    this.thetaController = thetaController;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public ChassisVelocities calculateSpeeds(Pose2d currentPose, Pose2d targetPose) {
    return new ChassisVelocities(
        xController.calculate(currentPose.getX(), targetPose.getX()),
        yController.calculate(currentPose.getY(), targetPose.getY()),
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()));
  }
}
