package frc.robot.autos;

import org.wpilib.math.geometry.Transform2d;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.util.PoseErrorTolerance;

public class AutoBlocks {
  public static final PoseErrorTolerance APPROACH_REEF_TOLERANCE = new PoseErrorTolerance(0.6, 10);
  public static final AutoConstraintOptions BASE_CONSTRAINTS = new AutoConstraintOptions();
  public static final AutoConstraintOptions LOLLIPOP_RACE_CONSTRAINTS = new AutoConstraintOptions();
  public static final Transform2d LOLLIPOP_OFFSET = new Transform2d();
}

