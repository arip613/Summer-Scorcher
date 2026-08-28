package frc.robot.IndexerSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class Indexer extends StateMachine<Indexer.IndexerState> {
  public enum IndexerState { OFF, INTAKE, FEED, REVERSE }

  public static final double INTAKE_POWER = -0.9;
    public static final double FEED_POWER = -1.0;
  public static final double REVERSE_POWER = -0.5;

  private final TalonFX indexerA;
  private final TalonFX indexerB;
  private double dutyPercent = FEED_POWER;

  public Indexer(TalonFX indexerA, TalonFX indexerB) {
    super(SubsystemPriority.DEPLOY, IndexerState.OFF);
     var cfg = new TalonFXConfiguration();

    cfg.CurrentLimits = new CurrentLimitsConfigs()
				.withSupplyCurrentLimit(60)
				.withSupplyCurrentLimitEnable(true)
				.withStatorCurrentLimit(80)
				.withStatorCurrentLimitEnable(true);
		indexerA.getConfigurator().apply(cfg);
    indexerB.getConfigurator().apply(cfg);
    
    this.indexerA = indexerA;
    this.indexerB = indexerB;
  }

  public void intake() { setStateFromRequest(IndexerState.INTAKE); }
  public void feed() { setStateFromRequest(IndexerState.FEED); }
  public void reverse() { setStateFromRequest(IndexerState.REVERSE); }
  public void stop() { setStateFromRequest(IndexerState.OFF); }

  /**
   * Set an arbitrary duty cycle percent and enter FEED state.
   * Percent is clamped to [-1.0, 1.0].
   */
  public void setDutyPercent(double percent) {
    dutyPercent = Math.max(-1.0, Math.min(1.0, percent));
    setStateFromRequest(IndexerState.FEED);
  }

  @Override
  protected IndexerState getNextState(IndexerState current) { return current; }

  @Override
  protected void afterTransition(IndexerState newState) {
    switch (newState) {
      case OFF -> setBoth(new DutyCycleOut(0.0));
      case INTAKE -> setBoth(new DutyCycleOut(INTAKE_POWER));
      case FEED -> setBoth(new DutyCycleOut(dutyPercent));
      case REVERSE -> setBoth(new DutyCycleOut(REVERSE_POWER));
    }
  }

  private void setBoth(DutyCycleOut request) {
    indexerA.setControl(request);
    indexerB.setControl(request);
  }
}
