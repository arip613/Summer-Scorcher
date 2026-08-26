package frc.robot.IndexerSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.system.Timer;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;


public class Hopper extends StateMachine<Hopper.HopperState> {
  public enum HopperState { OFF, INTAKE, FEED, REVERSE, PULSE }

  public static final double INTAKE_POWER = 1;
  public static final double FEED_POWER = 1;
  public static final double REVERSE_POWER = -0.5;

  public static final double PULSE_ON_POWER = 1;   
  public static final double PULSE_OFF_POWER = 0.0;  
  public static final double PULSE_ON_TIME = 0.01;   
  public static final double PULSE_OFF_TIME = 0.005;  
  private final TalonFX Hopper;
  private final Timer pulseTimer = new Timer();
  private boolean pulseOn = true;
  private double dutyPercent = FEED_POWER;

  public Hopper(TalonFX Hopper) {
    super(SubsystemPriority.DEPLOY, HopperState.OFF);
    var cfg = new TalonFXConfiguration();
		cfg.CurrentLimits = new CurrentLimitsConfigs()
				.withSupplyCurrentLimit(40)
				.withSupplyCurrentLimitEnable(true)
				.withStatorCurrentLimit(60)
				.withStatorCurrentLimitEnable(true);
		Hopper.getConfigurator().apply(cfg);
    Hopper.getConfigurator().apply(cfg);
    this.Hopper = Hopper;
  }

  public void intake() { setStateFromRequest(HopperState.INTAKE); }
  public void feed() { setStateFromRequest(HopperState.FEED); }
  public void reverse() { setStateFromRequest(HopperState.REVERSE); }
  public void stop() { setStateFromRequest(HopperState.OFF); }

  public void pulse() { setStateFromRequest(HopperState.PULSE); }

  public void setDutyPercent(double percent) {
    dutyPercent = Math.max(-1.0, Math.min(1.0, percent));
    setStateFromRequest(HopperState.FEED);
  }

  @Override
  protected HopperState getNextState(HopperState current) { return current; }

  @Override
  protected void afterTransition(HopperState newState) {
    switch (newState) {
      case OFF -> Hopper.setControl(new DutyCycleOut(0.0));
      case INTAKE -> Hopper.setControl(new DutyCycleOut(INTAKE_POWER));
  case FEED -> Hopper.setControl(new DutyCycleOut(dutyPercent));
      case REVERSE -> Hopper.setControl(new DutyCycleOut(REVERSE_POWER));
      case PULSE -> {
        pulseTimer.restart();
        pulseOn = true;
        Hopper.setControl(new DutyCycleOut(PULSE_ON_POWER));
      }
    }
  }

  @Override
  protected void collectInputs() {
    if (getState() == HopperState.PULSE) {
      double elapsed = pulseTimer.get();
      if (pulseOn && elapsed >= PULSE_ON_TIME) {
        pulseOn = false;
        pulseTimer.restart();
        Hopper.setControl(new DutyCycleOut(PULSE_OFF_POWER));
      } else if (!pulseOn && elapsed >= PULSE_OFF_TIME) {
        pulseOn = true;
        pulseTimer.restart();
        Hopper.setControl(new DutyCycleOut(PULSE_ON_POWER));
      }
    }
  }
}
