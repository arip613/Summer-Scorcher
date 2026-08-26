package frc.robot.autos;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

/**
 * Minimal placeholder subsystem for autos. Autonomous is created directly in Robot.
 */
public class Autos extends LifecycleSubsystem {
  public Autos() {
    super(SubsystemPriority.AUTOS);
  }

  public Command getAutoCommand() {
    return Commands.none();
  }
}
