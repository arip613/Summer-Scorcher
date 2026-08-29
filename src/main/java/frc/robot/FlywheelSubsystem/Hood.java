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
		cfg.CurrentLimits = new CurrentLimitsConfigs()
				.withSupplyCurrentLimit(20.0)
				.withSupplyCurrentLimitEnable(true)
				.withStatorCurrentLimit(40.0)
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

	}

	public double getAngleRadians() {
		return Math.toRadians(getAngleDegrees());
	}
}
