package frc.robot.FlywheelSubsystem;

import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class DrumStateMachine extends StateMachine<DrumStateMachine.State> {
  /** TUNING hands the motors to {@link DrumTuner}; this state machine commands nothing in it. */
  public enum State { OFF, SPIN_RPM, TUNING }

  private final Drum drum;
  private double targetRpm = 0.0;

  public DrumStateMachine(Drum drum) {
    super(SubsystemPriority.DEPLOY, State.OFF);
    this.drum = drum;
  }

  public void requestOff() { setStateFromRequest(State.OFF); }
  public void requestRpm(double rpm) {
    this.targetRpm = rpm;
    setStateFromRequest(State.SPIN_RPM);
  }
  public void requestTuning() { setStateFromRequest(State.TUNING); }

  @Override
  protected State getNextState(State current) { return current; }

  @Override
  protected void collectInputs() {
    drum.periodicTelemetry();

    if (getState() == State.SPIN_RPM) {
      drum.spinDrum(targetRpm);
    }
  }

  @Override
  protected void afterTransition(State newState) {
    switch (newState) {
      case OFF -> drum.stop();
      case SPIN_RPM -> drum.spinDrum(targetRpm);
      case TUNING -> drum.stop();
    }
  }
}
