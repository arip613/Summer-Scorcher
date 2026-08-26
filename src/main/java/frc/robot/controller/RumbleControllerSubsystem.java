package frc.robot.controller;

import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.system.Timer;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class RumbleControllerSubsystem extends StateMachine<RumbleControllerState> {
  private final Timer matchTimer = new Timer();
  private final GenericHID controller;
  @SuppressWarnings("unused")
  private final boolean matchTimeRumble;
  public static final double MATCH_DURATION_TELEOP = 135;

  @Override
  public void teleopInit() {
    matchTimer.reset();
    matchTimer.start();
  }

  @Override
  public void disabledInit() {
    matchTimer.stop();
  }

  public RumbleControllerSubsystem(CommandGamepad controller, boolean matchTimeRumble) {
    this(controller.getHID(), matchTimeRumble);

    if (matchTimeRumble) {
      var rumbleCommand = Commands.runOnce(() -> rumbleRequest());
      new Trigger(() -> matchTimer.hasElapsed(MATCH_DURATION_TELEOP - 90)).onTrue(rumbleCommand);
      new Trigger(() -> matchTimer.hasElapsed(MATCH_DURATION_TELEOP - 60)).onTrue(rumbleCommand);
      new Trigger(() -> matchTimer.hasElapsed(MATCH_DURATION_TELEOP - 30)).onTrue(rumbleCommand);
    }
  }

  public RumbleControllerSubsystem(GenericHID controller, boolean matchTimeRumble) {
    super(SubsystemPriority.RUMBLE_CONTROLLER, RumbleControllerState.OFF);
    this.controller = controller;
    this.matchTimeRumble = matchTimeRumble;
  }

  public void rumbleRequest() {
    if (!RobotState.isAutonomous()) {
      setStateFromRequest(RumbleControllerState.ON);
    }
  }

  @Override
  protected RumbleControllerState getNextState(RumbleControllerState currentState) {
    return switch (currentState) {
      case ON -> timeout(0.5) ? RumbleControllerState.OFF : currentState;
      case OFF -> currentState;
    };
  }

  @Override
  protected void afterTransition(RumbleControllerState newState) {
    switch (newState) {
      case ON -> {
        setBothRumble(1);
      }
      case OFF -> {
        setBothRumble(0);
      }
    }
  }

  /** RumbleType no longer has a combined value, so drive both motors together. */
  private void setBothRumble(double value) {
    controller.setRumble(RumbleType.LEFT_RUMBLE, value);
    controller.setRumble(RumbleType.RIGHT_RUMBLE, value);
  }
}
