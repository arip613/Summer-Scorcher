package frc.robot.FlywheelSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.smartdashboard.SmartDashboard;


public class Hood {
	private static final double GEAR_RATIO = 84.0; 
	private static final double MIN_DEG = -40;
	private static final double MAX_DEG = 40;
	/**
	 * How close the hood has to be to its target to count as ready.
	 *
	 * <p>Was 0.1 degrees, which the hood cannot physically hold. Slot0 is pure kP with no kI and no
	 * kG, so it settles wherever the proportional term balances gravity and friction: measured at
	 * -39.54 against a -40 target in match AZGLE4_Q12, a steady 0.46 degrees short, held for 2.4
	 * seconds. isAtGoal() was false the entire time.
	 *
	 * <p>That only ever broke passing, because passReady is the one gate that checks the hood --
	 * the shot gate deliberately does not -- so the pass could never fire no matter how long the
	 * driver held the trigger. 1.0 gives about 2x margin over the observed droop while still being
	 * tight enough to matter for shot angle.
	 *
	 * <p>The droop itself is the real defect. Adding kG to Slot0 would remove it and let this go
	 * back to being tight, but that is tuning that needs the robot.
	 */
	private static final double AT_GOAL_TOL_DEG = 1.0;

	private final TalonFX motor;
	private final MotionMagicTorqueCurrentFOC mmRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
	private double lastTargetDeg = 0.0;

	public Hood(TalonFX motor) {
		this.motor = motor;

		var cfg = new TalonFXConfiguration();
		cfg.Slot0 = new Slot0Configs().withKP(50).withKI(0).withKD(0);
		// Jerk turns the trapezoid into an S-curve. Without it Motion Magic changes acceleration
		// instantaneously, so the hood slams from full deceleration to zero at the setpoint -- that
		// corner is what the abrupt stop is. 1200 rot/s^3 is 10x the acceleration, which rounds the
		// entry and exit of each ramp over about 0.1s (accel / jerk) and costs roughly that much on
		// a move. Lower it for a softer stop at the price of a slower hood.
		cfg.MotionMagic =
				new MotionMagicConfigs()
						.withMotionMagicCruiseVelocity(100.0)
						.withMotionMagicAcceleration(120)
						.withMotionMagicJerk(1200);
		// Raised from 40A stator / 20A supply. The hood repeatedly parked at -6.5 degrees against
		// targets of -19.6, -32.7 and -40 in match AZGLE4_E14 -- always the same angle, wildly
		// different commands -- while at other points in the same match it sailed through to -36.4.
		// Something catches around -6.5 and 40A could not reliably break it, which is why the pass
		// never fired: passReady needs hood.isAtGoal() and the hood never arrived.
		//
		// Stator is what produces torque at a stall, so that is the one that matters here. Supply
		// goes up with it so it does not become the new limit once duty climbs while pushing.
		//
		// This buys torque to get through a mechanical catch; it does not remove the catch. Watch
		// Hood/StatorCurrent: sitting pinned at 60A is the hood grinding against something, not
		// working, and wants fixing mechanically rather than with more current.
		cfg.CurrentLimits = new CurrentLimitsConfigs()
				.withSupplyCurrentLimit(30.0)
				.withSupplyCurrentLimitEnable(true)
				.withStatorCurrentLimit(60.0)
				.withStatorCurrentLimitEnable(true);

		motor.getConfigurator().apply(cfg);

		motor.setPosition(0.0);
	}

	public void setAngleDegrees(double degrees) {
		double clamped = Math.max(MIN_DEG, Math.min(MAX_DEG, degrees));
		lastTargetDeg = clamped;
		double rotations = (clamped / 360.0) * GEAR_RATIO;
		motor.setControl(mmRequest.withPosition(rotations));
	}

	public void dutyCycle(double perce) {
		motor.setControl(new DutyCycleOut(perce));
	}


	public void hold() {
		double currentRot = motor.getPosition().getValueAsDouble();
		motor.setControl(mmRequest.withPosition(currentRot));
	}

	public void runDutyCycle(double percent) {
		motor.setControl(new DutyCycleOut(percent));
	}

	/** Coast / stop the hood motor with no active control. */
	public void stopMotor() {
		motor.setControl(new DutyCycleOut(0.0));
	}

	public double getAngleDegrees() {
		double rotations = motor.getPosition().getValueAsDouble();
		return (rotations / GEAR_RATIO) * 360.0;
	}

	public boolean isAtGoal() {
		return Math.abs(getAngleDegrees() - lastTargetDeg) <= AT_GOAL_TOL_DEG;
	}

	public void periodicTelemetry() {
		SmartDashboard.putNumber("Hood/AngleDeg", getAngleDegrees());
		SmartDashboard.putNumber("Hood/TargetDeg", lastTargetDeg);
		SmartDashboard.putBoolean("Hood/AtGoal", isAtGoal());
		SmartDashboard.putNumber("Hood/ErrorDeg", lastTargetDeg - getAngleDegrees());

		// A stall and a mechanism that is merely slow look identical in position alone, which is
		// what made the E14 hood problem take a log dive to find. Pinned current with standing
		// error is a stall; that reads off these two immediately.
		SmartDashboard.putNumber("Hood/StatorCurrent", readCurrent(true));
		SmartDashboard.putNumber("Hood/SupplyCurrent", readCurrent(false));
	}

	/** Guarded because a motor that is absent or off the bus returns null rather than a value. */
	private double readCurrent(boolean stator) {
		try {
			var signal = stator ? motor.getStatorCurrent() : motor.getSupplyCurrent();
			Double value = signal.getValueAsDouble();
			return value == null ? 0.0 : value;
		} catch (Exception ex) {
			return 0.0;
		}
	}

	public double getAngleRadians() {
		return Math.toRadians(getAngleDegrees());
	}
}
