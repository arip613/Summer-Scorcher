package frc.robot.autos;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

/**
 * Minimal stub of AutoCommands to satisfy compilation after removing superstructure features.
 * All methods return no-op commands.
 */
public class AutoCommands {
  public AutoCommands() {}

  public Command stowRequest() { return Commands.none(); }
  public Command preloadCoralCommand() { return Commands.none(); }
  public boolean alignedForScore() { return false; }
  public Command intakeCoralHorizontalCommand() { return Commands.none(); }
  public Command lollipopApproachCommand() { return Commands.none(); }
  public Command intakeLollipopCommand() { return Commands.none(); }
  public Command homeDeployCommand() { return Commands.none(); }
  public Command waitForIntakeDone() { return Commands.none(); }
  public Command waitForLollipopIntakeDone() { return Commands.none(); }
  public Command waitForElevatorAndArmNearLollipop() { return Commands.none(); }
  public Command l4ApproachCommand(Object pipe, Object scoringSide) { return Commands.none(); }
  public Command l3ApproachCommand(Object scoringSide) { return Commands.none(); }
  public Command l2ApproachCommand(Object pipe, Object scoringSide) { return Commands.none(); }
  public Command l2LineupCommand(Object scoringSide) { return Commands.none(); }
  public Command l4LeftReleaseCommand(Object pipe, Object scoringSide) { return Commands.none(); }
  public Command l3LeftReleaseCommand() { return Commands.none(); }
  public Command moveToStartingPositionCommand() { return Commands.none(); }
  public Command waitForReleaseCommand() { return Commands.none(); }
  public Command waitForAlignedForScore() { return Commands.none(); }
  public Command groundIntakeToL4Command() { return Commands.none(); }
}
