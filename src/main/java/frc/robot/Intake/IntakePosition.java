package frc.robot.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.command2.Commands;
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
			/** Looping staged agitation: 20% of travel, 50%, all the way down, all the way up. */
			PULSE,
			/** Sweeps the full travel end to end, for use while a shot is being taken. */
			SWING,
			/** Retract with no motion profile -- pulls the arm in as hard as the limits allow. */
			STOW
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

	/**
	 * PULSE agitation stages: how far to raise the arm from the deployed end, as a fraction of the
	 * full travel, with how long to hold each before moving on. 0.0 is fully deployed, 1.0 is fully
	 * retracted. The sequence tilts up a fifth, then half, drops back down, then sweeps all the way
	 * to retracted -- and then repeats from the start for as long as PULSE is held.
	 */
	private static final double[] SWING_STAGE_FRACTIONS = {0.20, 0.50, 0.00, 1.00};

	/**
	 * Advance a stage once the arm is within this many rotations of the stage target, rather than
	 * after a fixed time. The stages cover very different distances -- 2.2 rotations for the first,
	 * 11 for the last -- so a single duration either cut the long moves short or wasted time on the
	 * short ones.
	 */
	private static final double STAGE_TOLERANCE_ROTATIONS = 0.40;

	/**
	 * Give up on a stage after this long and move on anyway, so a jammed or stalled arm cannot
	 * freeze the sequence partway through.
	 */
	private static final double STAGE_TIMEOUT_SECONDS = 1.0;

	/** SWING sweeps the whole travel end to end; this is how long one leg takes. */
	private static final double FULL_SWING_LEG_SECONDS = 0.35;

	private final TalonFX motor;
	private final MotionMagicTorqueCurrentFOC mmRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);

	/**
	 * Straight position control with no motion profile, used only for the very first deploy so the
	 * arm gets down as fast as the gains and the current limit allow. Everything after that goes
	 * back through Motion Magic's velocity/acceleration limits.
	 */
	private final PositionTorqueCurrentFOC fastRequest = new PositionTorqueCurrentFOC(0).withSlot(0);
	private boolean hasDeployedOnce = false;

	/**
	 * Agitation moves at twice the configured Motion Magic rate. Dynamic Motion Magic carries the
	 * velocity and acceleration on the request itself, so this does not disturb the config that
	 * every other motion uses -- no runtime reconfiguration, no need to set it back.
	 */
	private static final double AGITATE_SPEED_MULTIPLIER = 2.0;
	private static final double BASE_CRUISE_VELOCITY = 20.0;
	private static final double BASE_ACCELERATION = 30.0;
	private final DynamicMotionMagicTorqueCurrentFOC agitateRequest =
			new DynamicMotionMagicTorqueCurrentFOC(
					0,
					BASE_CRUISE_VELOCITY * AGITATE_SPEED_MULTIPLIER,
					BASE_ACCELERATION * AGITATE_SPEED_MULTIPLIER)
				.withSlot(0);

	private double deployRotations = DEFAULT_DEPLOY_ROTATIONS;
	private double lastCommandedRotations = Double.NaN;

	// Pulse/swing state helpers
	private double lastPulseToggleTime = -1.0;
	private boolean pulseDeployed = false;
	private int swingStage = 0;
	private double swingStageStart = -1.0;

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

		// Dashboard buttons to exercise the arm without holding a controller trigger. These drive
		// the same states the shot path uses, so what you see here is what happens in a match.
		SmartDashboard.putData("Intake/Test/RunAgitation",
				Commands.runOnce(this::pulse).ignoringDisable(false).withName("Intake/Agitate"));
		SmartDashboard.putData("Intake/Test/RunSwing",
				Commands.runOnce(this::swing).ignoringDisable(false).withName("Intake/Swing"));
		SmartDashboard.putData("Intake/Test/Deploy",
				Commands.runOnce(this::deploy).ignoringDisable(false).withName("Intake/Deploy"));
		SmartDashboard.putData("Intake/Test/Retract",
				Commands.runOnce(this::retract).ignoringDisable(false).withName("Intake/Retract"));
		SmartDashboard.putData("Intake/Test/Stop",
				Commands.runOnce(this::stopPulse).ignoringDisable(true).withName("Intake/Stop"));
}

	/** Commands a position and records it, so telemetry can show target vs actual. */
	private void goTo(double rotations) {
		lastCommandedRotations = rotations;
		motor.setControl(mmRequest.withPosition(rotations));
	}

	/**
	 * Commands a position with no motion profile at all -- no cruise velocity, no acceleration
	 * limit. The arm moves as hard as kP and the 80 A stator limit allow.
	 */
	private void goToFast(double rotations) {
		lastCommandedRotations = rotations;
		motor.setControl(fastRequest.withPosition(rotations));
	}

	/** Commands a position at the agitation rate: same profile shape, twice the speed. */
	private void goToAgitate(double rotations) {
		lastCommandedRotations = rotations;
		motor.setControl(agitateRequest.withPosition(rotations));
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

	/**
	 * Looping staged agitation: 20% of travel, then 50%, then all the way down, then all the way
	 * up, repeating until the state changes. Safe to call every loop -- the request is idempotent
	 * and the cycle advances on its own.
	 */
	public void pulse() { setStateFromRequest(State.PULSE); }

	/** Sweep the full travel end to end, for use while shooting. */
	public void swing() { setStateFromRequest(State.SWING); }

	/** Pull the arm all the way in with no motion profile -- as hard and fast as the limits allow. */
	public void forceStow() { setStateFromRequest(State.STOW); }

	/**
	 * Position raised from the deployed end by a fraction of the full travel. 0 = still fully
	 * deployed, 1 = fully retracted. So 0.20 tilts the arm up by a fifth of its range.
	 */
	private double raisedByFraction(double fraction) {
		return deployRotations - fraction * (deployRotations - RETRACT_ROTATIONS);
	}

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
				// Staged rather than a timer toggle: 20% of travel, then 50%, then all the way
				// down, then all the way up -- and then around again for as long as the state is
				// held. Each stage ends on whichever comes first, reaching the target or the stage
				// timeout, so a stage that cannot physically complete still advances.
				//
				// The cycle must wrap. It used to stop on the last stage, which left the arm parked
				// fully retracted with swingStage stuck at the end. Re-requesting PULSE did not
				// restart it either, because setStateFromRequest is a no-op when already in the
				// state and the stage counter only resets on a transition -- so the agitation ran
				// exactly once per entry into PULSE and never again.
				double now = Timer.getTimestamp();
				double stageTarget = raisedByFraction(SWING_STAGE_FRACTIONS[swingStage]);
				boolean reached =
						Math.abs(currentRotations() - stageTarget) <= STAGE_TOLERANCE_ROTATIONS;
				boolean timedOut = now - swingStageStart >= STAGE_TIMEOUT_SECONDS;
				if (reached || timedOut) {
					swingStage = (swingStage + 1) % SWING_STAGE_FRACTIONS.length;
					swingStageStart = now;
					goToAgitate(raisedByFraction(SWING_STAGE_FRACTIONS[swingStage]));
				}
			} else if (getState() == State.SWING) {
				// Sweep end to end for as long as the state is held.
				double now = Timer.getTimestamp();
				if (now - lastPulseToggleTime >= FULL_SWING_LEG_SECONDS) {
					pulseDeployed = !pulseDeployed;
					goToAgitate(pulseDeployed ? deployRotations : RETRACT_ROTATIONS);
					lastPulseToggleTime = now;
				}
			}

			double actual = currentRotations();
			SmartDashboard.putNumber("Intake/Position", actual);
			SmartDashboard.putNumber("Intake/Target", lastCommandedRotations);
			SmartDashboard.putNumber("Intake/ErrorRotations", lastCommandedRotations - actual);
			SmartDashboard.putString("Intake/State", getState().toString());
			SmartDashboard.putNumber("Intake/SwingStage", swingStage);
		}

	@Override
	protected void afterTransition(State newState) {
		switch (newState) {
			case OFF     -> {
				lastCommandedRotations = Double.NaN;
				motor.setControl(new CoastOut());
			}
			case DEPLOY  -> {
				// First deploy of the session skips the motion profile so the arm slams down; every
				// deploy after that uses Motion Magic's limits.
				if (hasDeployedOnce) {
					goTo(deployRotations);
				} else {
					hasDeployedOnce = true;
					goToFast(deployRotations);
				}
			}
			case RETRACT -> goTo(RETRACT_ROTATIONS);
			case STOW    -> goToFast(RETRACT_ROTATIONS);
			case SHOOTER -> goTo(-0.6);
			case PULSE   -> {
				swingStage = 0;
				swingStageStart = Timer.getTimestamp();
				goToAgitate(raisedByFraction(SWING_STAGE_FRACTIONS[0]));
			}
			case SWING   -> {
				pulseDeployed = true;
				lastPulseToggleTime = Timer.getTimestamp();
				goToAgitate(deployRotations);
			}
		}
	}
	
}
