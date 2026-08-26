package frc.robot.FlywheelSubsystem;

import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;


public class HoodStateMachine extends StateMachine<HoodStateMachine.State> {
  public enum State { OFF, DEGREE_TARGET }

  private final Hood hood;
  private double targetDegrees = 0.0;

  public HoodStateMachine(Hood hood) {
    super(SubsystemPriority.DEPLOY, State.OFF);
    this.hood = hood;
  }


  public void requestOff() { setStateFromRequest(State.OFF); }
  public void requestDegrees(double degrees) {
    this.targetDegrees = degrees;
    if (getState() == State.DEGREE_TARGET) {
      hood.setAngleDegrees(targetDegrees);
      return;
    }
    setStateFromRequest(State.DEGREE_TARGET);
  }

  @Override
  protected State getNextState(State current) { return current; }

  @Override
  protected void collectInputs() {
    hood.periodicTelemetry();
  }

  @Override
  protected void afterTransition(State newState) {
    switch (newState) {
      case OFF -> hood.hold();
      case DEGREE_TARGET -> hood.setAngleDegrees(targetDegrees);
    }
  }
}
