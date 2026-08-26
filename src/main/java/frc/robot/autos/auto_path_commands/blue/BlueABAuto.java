package frc.robot.autos.auto_path_commands.blue;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.command2.Command;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;

/**
 * Minimal blue auto: drive from point A to point B.
 */
public class BlueABAuto extends BaseAuto {
  public BlueABAuto(Trailblazer trailblazer) {
    super(trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R1_AND_B1_FORWARD.bluePose;
  }

  @Override
  protected Command createAutoCommand() {
    var start = Points.START_R1_AND_B1_FORWARD.bluePose;
    var end = new Pose2d(15.0, start.getY(), Rotation2d.kZero);
    var constraints = new AutoConstraintOptions();

    return trailblazer.followSegment(
        new AutoSegment(constraints, new AutoPoint(start), new AutoPoint(end)));
  }
}
