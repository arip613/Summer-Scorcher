package frc.robot.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.system.Timer;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakePosition extends StateMachine<IntakePosition.State> {
	public enum State {
			OFF,
			DEPLOY,
			RETRACT,
			SHOOTER,
			PULSE
	}

	private static final double DEPLOY_ROTATIONS = 9.75;

	private final TalonFX motor;
	private final MotionMagicTorqueCurrentFOC mmRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);

	// Pulse state helpers
	private double lastPulseToggleTime = -1.0;
	private boolean pulseDeployed = false;

	public IntakePosition(TalonFX motor) {
		super(SubsystemPriority.DEPLOY, State.OFF);
		this.motor = motor;
		

		var cfg = new TalonFXConfiguration();
cfg.Slot0 = new Slot0Configs()
    .withKP(30).withKI(0).withKD(0)
    .withKG(0.3)
    .withKS(0.1);
cfg.MotionMagic = new MotionMagicConfigs()
    .withMotionMagicCruiseVelocity(20)
    .withMotionMagicAcceleration(30);
cfg.CurrentLimits = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(60)
    .withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimit(80)
    .withStatorCurrentLimitEnable(true);
cfg.MotorOutput = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Brake);
	motor.getConfigurator().apply(cfg);
		motor.setPosition(0.0);	
}

	/** Move elevator to deployed position. */
	public void deploy()  { setStateFromRequest(State.DEPLOY); }
	/** Retract elevator back to boot/zero position. */
	public void retract() { setStateFromRequest(State.RETRACT); }

	/** Start pulsing: deploy/retract repeatedly every 1s. */
	public void pulse() { setStateFromRequest(State.PULSE); }

	public void shooter() { setStateFromRequest(State.SHOOTER); }


	/** Stop pulsing and go OFF. */
	public void stopPulse() { setStateFromRequest(State.OFF); }

	@Override
	protected State getNextState(State current) { return current; }

		@Override
		protected void collectInputs() {
			if (getState() == State.PULSE) {
				double now = Timer.getTimestamp();
				if (lastPulseToggleTime < 0) {
					motor.setControl(mmRequest.withPosition(DEPLOY_ROTATIONS));
					pulseDeployed = true;
					lastPulseToggleTime = now;
				} else if (now - lastPulseToggleTime >= 0.3) {
					if (pulseDeployed) {
						motor.setControl(mmRequest.withPosition(-0.85));
						pulseDeployed = false;
					} else {
						motor.setControl(mmRequest.withPosition(DEPLOY_ROTATIONS));
						pulseDeployed = true;
					}
					lastPulseToggleTime = now;
				}
			}
		}

	@Override
	protected void afterTransition(State newState) {
		switch (newState) {
			case OFF     -> motor.setControl(new CoastOut());
			case DEPLOY  -> motor.setControl(mmRequest.withPosition(DEPLOY_ROTATIONS));
			case RETRACT -> motor.setControl(mmRequest.withPosition(-0.85));
			case SHOOTER -> motor.setControl(mmRequest.withPosition(-0.6));
			case PULSE   -> {
				motor.setControl(mmRequest.withPosition(DEPLOY_ROTATIONS));
				pulseDeployed = true;
				lastPulseToggleTime = Timer.getTimestamp();
			}
		}
	}
	
}
