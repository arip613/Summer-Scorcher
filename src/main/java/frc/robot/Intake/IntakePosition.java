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

import org.wpilib.smartdashboard.SmartDashboard;
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

	// Lower number = arm sits higher. Also used as the "down" end of the PULSE cycle.
	// Live-adjustable via Intake/Tune/DeployRotations so the height can be dialled in without a
	// deploy cycle; this is only the power-on default.
	//
	// This is where the arm sits on teleop enable and while the left trigger is held. Lowered by
	// 0.15 rotations from 9.75 for about a quarter inch, at the ~0.6 rotations/inch implied by the
	// -1.26..9.75 sweep.
	private static final double DEFAULT_DEPLOY_ROTATIONS = 9.75;
	private static final String DEPLOY_KEY = "Intake/Tune/DeployRotations";

	/**
	 * The height the arm is held at in normal operation. Raised by 0.30 rotations from the previous
	 * -0.96 for roughly half an inch of extra clearance -- the full -0.96..9.75 sweep is 10.71
	 * rotations, which works out near 0.6 rotations per inch of arc, so 0.30 is about 0.5 in. There
	 * is no gear ratio in this class to convert exactly; adjust if it measures off.
	 */
	private static final double RETRACT_ROTATIONS = -1.26;

	private final TalonFX motor;
	private final MotionMagicTorqueCurrentFOC mmRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);

	private double deployRotations = DEFAULT_DEPLOY_ROTATIONS;
	private double lastCommandedRotations = Double.NaN;

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
		// No homing, no absolute encoder: zero is wherever the arm physically rests at power-on, so
		// every position below is relative to that. If the arm sits differently at boot, the same
		// number lands at a different height -- see Intake/Position to check.
		motor.setPosition(0.0);

		// putNumber, not setDefaultNumber, and deliberately NOT persistent: the constant above must
		// win on every boot. A persisted value silently outranked the source, so edits to the
		// constant were deployed and then ignored. Live edits still apply for the rest of a session.
		SmartDashboard.putNumber(DEPLOY_KEY, DEFAULT_DEPLOY_ROTATIONS);
		deployRotations = DEFAULT_DEPLOY_ROTATIONS;
}

	/** Commands a position and records it, so telemetry can show target vs actual. */
	private void goTo(double rotations) {
		lastCommandedRotations = rotations;
		motor.setControl(mmRequest.withPosition(rotations));
	}

	private double currentRotations() {
		try {
			return motor.getPosition().getValueAsDouble();
		} catch (Exception ex) {
			return 0.0;
		}
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
			// Pick up live edits to the deploy height and re-command immediately if the arm is
			// currently deployed, so the effect is visible without releasing the trigger.
			double wanted = SmartDashboard.getNumber(DEPLOY_KEY, DEFAULT_DEPLOY_ROTATIONS);
			if (wanted != deployRotations) {
				deployRotations = wanted;
				if (getState() == State.DEPLOY) {
					goTo(deployRotations);
				}
			}

			if (getState() == State.PULSE) {
				double now = Timer.getTimestamp();
				if (lastPulseToggleTime < 0) {
					goTo(deployRotations);
					pulseDeployed = true;
					lastPulseToggleTime = now;
				} else if (now - lastPulseToggleTime >= 0.3) {
					if (pulseDeployed) {
						goTo(-0.85);
						pulseDeployed = false;
					} else {
						goTo(deployRotations);
						pulseDeployed = true;
					}
					lastPulseToggleTime = now;
				}
			}

			double actual = currentRotations();
			SmartDashboard.putNumber("Intake/Position", actual);
			SmartDashboard.putNumber("Intake/Target", lastCommandedRotations);
			SmartDashboard.putNumber("Intake/ErrorRotations", lastCommandedRotations - actual);
			SmartDashboard.putString("Intake/State", getState().toString());
		}

	@Override
	protected void afterTransition(State newState) {
		switch (newState) {
			case OFF     -> {
				lastCommandedRotations = Double.NaN;
				motor.setControl(new CoastOut());
			}
			case DEPLOY  -> goTo(deployRotations);
			case RETRACT -> goTo(RETRACT_ROTATIONS);
			case SHOOTER -> goTo(-0.6);
			case PULSE   -> {
				goTo(deployRotations);
				pulseDeployed = true;
				lastPulseToggleTime = Timer.getTimestamp();
			}
		}
	}
	
}
