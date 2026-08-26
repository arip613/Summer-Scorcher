package frc.robot.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;


public class intaker extends StateMachine<intaker.State> {
	public enum State { OFF, INTAKE, FEED, REVERSE, AUTO }

	private static final double INTAKE_POWER = 12;
	private static final double FEED_POWER = 12;
	private static final double REVERSE_POWER = -12;
	private static final double AUTO = 11;

	private final TalonFX motorA;
	private final TalonFX motorB;


	public intaker(TalonFX motorA, TalonFX motorB) {
		super(SubsystemPriority.DEPLOY, State.OFF);
		this.motorA = motorA;
		this.motorB = motorB;

		var cfg = new TalonFXConfiguration();
		cfg.CurrentLimits = new CurrentLimitsConfigs()
				.withSupplyCurrentLimit(30) //45
				.withSupplyCurrentLimitEnable(true)
				.withStatorCurrentLimit(50) //50
				.withStatorCurrentLimitEnable(true);
		motorA.getConfigurator().apply(cfg);
		motorB.getConfigurator().apply(cfg);

		motorA.getConfigurator().apply(
				new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
		motorB.getConfigurator().apply(
				new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
	}

	public void intake() { setStateFromRequest(State.INTAKE); }
	public void feed() { setStateFromRequest(State.FEED); }
	public void reverse() { setStateFromRequest(State.REVERSE); }
	public void stop() { setStateFromRequest(State.OFF);  }
	public void auto() { setStateFromRequest(State.AUTO); }

	@Override
	protected State getNextState(State current) { return current; }

	@Override
	protected void afterTransition(State newState) {
		switch (newState) {
			case OFF -> setBoth(new VoltageOut(0.0));
			case INTAKE -> setBoth(new VoltageOut(INTAKE_POWER));
			case FEED -> setBoth(new VoltageOut(FEED_POWER));
			case REVERSE -> setBoth(new VoltageOut(REVERSE_POWER));
			case AUTO -> setBoth(new VoltageOut(AUTO));
	}
}

	private void setBoth(VoltageOut request) {
		motorA.setControl(request);
		motorB.setControl(request);
	}
}
