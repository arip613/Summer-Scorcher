package frc.robot.autos.auto_path_commands.blue;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;

public class BlueDoNothingAuto extends BaseAuto {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(2, 57, 4, 30);

  public BlueDoNothingAuto(Trailblazer trailblazer) {
    super(trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R1_AND_B1.bluePose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(Points.START_R1_AND_B1.bluePose),
                new AutoPoint(new Pose2d(6.924, 7.292, Rotation2d.kZero)))));
  }
}
