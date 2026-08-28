package frc.robot.FlywheelSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.smartdashboard.SmartDashboard;

public class Drum {
  private final TalonFX a1, a2, a3, a4;

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);

  // VelocityTorqueCurrentFOC means every gain below is in AMPS, not volts:
  //   kS amps to break static friction, kV amps per rps (a drag model, NOT back-EMF),
  //   kA amps per rps^2, kP amps per rps of error.
  // Motor-spec or ReCalc voltage numbers do not transfer here.
  // Measured 2026-08-26 by DrumTuner FindKs: 7.79 A at first motion, 10.43 A at 20 RPM.
  // Two runs agreed within 3.5%. The old 0.3 was low by more than an order of magnitude.
  public static final double kS = 7.79;
  // Measured 2026-08-26 by DrumTuner StepTest at 2500 RPM: 13.96 A held steady, so
  // kV = (13.96 - 7.79) / (2500/60) = 0.148. Torque-per-amp is bus-voltage independent, so the
  // battery sag on that run does not bias this -- but overshoot/ripple from it are not trusted.
  public static final double kV = 0.148;
  public static final double kA = 0.0;
  // Raised from 11 -> 16 -> 20 by measuring shot recovery on 2026-08-27. Worst dip per ball fell
  // 337 -> 236 RPM: more gain pushes back earlier in the dip, so it never gets as deep, and balls
  // leave nearer the target speed. Recovery peaks around 63 A against the 80 A cap, so the loop is
  // not current-limited here.
  //
  // Tradeoff to know about: at 20 the steady-state error sits around +30..+100 RPM under load
  // rather than +/-40 centred on zero, so it crosses RPM_TOLERANCE (80) often and AtGoal chatters.
  // That repeatedly zeroes the shot-ready counter, which paces the feed. Currently that pacing is
  // helping consistency, but it is a side effect, not a design -- if the feed ever needs to run
  // continuously, RPM_TOLERANCE is the knob, not kP.
  public static final double kP = 20;
  // Only has an effect once kA is nonzero -- it is the setpoint the kA feedforward acts on.
  public static final double MAX_ACCEL_RPS2 = 350;

  public static final double SUPPLY_LIMIT = 60; //60
  public static final double STATOR_LIMIT = 80; //80
  public static final double TORQUE_CURRENT_LIMIT = 80; //80

  /** Fast enough that a 50 Hz control loop never reads the same sample twice. */
  private static final double VELOCITY_UPDATE_HZ = 100.0;

  public static final double RPM_TOLERANCE = 80;
  public static final double AT_GOAL_DEBOUNCE_TIME = 0.06;

  private static final double DRUM_OUTPUT_SIGN = -1.0;

  private static final InvertedValue DRUM_FORWARD = InvertedValue.CounterClockwise_Positive;
  private static final InvertedValue DRUM_REVERSED = InvertedValue.Clockwise_Positive;

  private final Debouncer atGoalDebounce =
      new Debouncer(AT_GOAL_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);

  private double targetRpm = 0.0;
  private boolean atGoal = false;
  private double liveKP = kP;
  private double liveKV = kV;
  private double liveKS = kS;
  private double liveKA = kA;

  private final TalonFX[] motors;

  /**
   * Velocity signals, refreshed together each loop. Reading getRotorVelocity() ad hoc returns
   * whatever each motor last reported at its own update rate, so the four samples are skewed in
   * time relative to each other -- which shows up as large phantom RpmSpread even when the drum is
   * one rigid shaft. refreshAll() takes them as a synchronized set.
   */
  private final StatusSignal<?>[] velocitySignals;
  private final BaseStatusSignal[] velocityRefresh;
  private final double[] motorRpm;

  /**
   * Rotor velocity picks up roughly +/-100 RPM of scatter whenever torque current is flowing, and
   * none of it when the drum is coasting -- so it is noise in the estimate, not the drum actually
   * changing speed (its inertia rules that out at ~15 A). Averaging 5 samples at 50 Hz costs about
   * 50 ms of lag on the at-goal signal and removes the scatter, which matters because RPM_TOLERANCE
   * is 80 and the raw noise alone was wider than that.
   *
   * <p>This only cleans up what the robot code sees. Each TalonFX still closes its own velocity
   * loop on its own raw signal on-motor.
   */
  private final LinearFilter rpmFilter = LinearFilter.movingAverage(5);
  private double filteredRpm = 0.0;
  private double rawRpm = 0.0;

  public Drum(TalonFX a1, TalonFX a2, TalonFX a3, TalonFX a4) {
    this.a1 = a1;
    this.a2 = a2;
    this.a3 = a3;
    this.a4 = a4;
    this.motors = new TalonFX[] {a1, a2, a3, a4};
    this.velocitySignals = new StatusSignal<?>[motors.length];
    this.velocityRefresh = new BaseStatusSignal[motors.length];
    this.motorRpm = new double[motors.length];
    for (int i = 0; i < motors.length; i++) {
      var signal = motors[i].getRotorVelocity();
      signal.setUpdateFrequency(VELOCITY_UPDATE_HZ);
      velocitySignals[i] = signal;
      velocityRefresh[i] = signal;
    }

    var cfg = new TalonFXConfiguration();

  cfg.Slot0 = new Slot0Configs()
    .withKS(kS)
    .withKV(kV)
    .withKA(kA)
    .withKP(kP);

    cfg.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(SUPPLY_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(STATOR_LIMIT)
        .withStatorCurrentLimitEnable(false);

    // In torque-current mode the stator limit above is not the operative clamp -- this is.
    // Phoenix 6 defaults these to +/-800 A, so without this the only real limit was the supply
    // limit. Symmetric because DRUM_OUTPUT_SIGN flips which direction accelerates the drum.
    cfg.TorqueCurrent = new TorqueCurrentConfigs()
        .withPeakForwardTorqueCurrent(TORQUE_CURRENT_LIMIT)
        .withPeakReverseTorqueCurrent(-TORQUE_CURRENT_LIMIT);

    a1.getConfigurator().apply(cfg);
    a2.getConfigurator().apply(cfg);
    a3.getConfigurator().apply(cfg);
    a4.getConfigurator().apply(cfg);
// charkie was here
// Ray smells terrible
// also like shit 
// this robot is so great
// Ray is a terrible human being
// Also a terrible human player (HP)
    a1.getConfigurator().apply(new MotorOutputConfigs().withInverted(DRUM_FORWARD));
    a2.getConfigurator().apply(new MotorOutputConfigs().withInverted(DRUM_FORWARD));
    a3.getConfigurator().apply(new MotorOutputConfigs().withInverted(DRUM_REVERSED));
    a4.getConfigurator().apply(new MotorOutputConfigs().withInverted(DRUM_REVERSED));

  SmartDashboard.putNumber("Drum/Tuning/kP", kP);
  SmartDashboard.putNumber("Drum/Tuning/kV", kV);
  SmartDashboard.putNumber("Drum/Tuning/kS", kS);
  SmartDashboard.putNumber("Drum/Tuning/kA", kA);
  }

  /**
   * Open-loop torque current, in amps at the drum's accelerating direction. Used by
   * {@link DrumTuner} to sweep for kS; bypasses the velocity loop entirely.
   */
  public void setTorqueCurrent(double amps) {
    targetRpm = 0.0;
    atGoal = false;
    var request = torqueRequest.withOutput(amps * DRUM_OUTPUT_SIGN);
    for (TalonFX motor : motors) {
      motor.setControl(request);
    }
  }

  /**
   * Mean per-motor torque current. Per-motor is the right quantity for deriving kS/kV/kA, because
   * Slot0 gains are applied by each controller individually.
   */
  public double getAvgTorqueCurrent() {
    double total = 0.0;
    for (TalonFX motor : motors) {
      total += Math.abs(safeTorque(motor));
    }
    return total / motors.length;
  }

  public double getSupplyCurrentTotal() {
    double total = 0.0;
    for (TalonFX motor : motors) {
      total += safeSupply(motor);
    }
    return total;
  }

  public void dutyCycle(double power) {
    targetRpm = 0.0;
    atGoal = false;
    var request = dutyCycleRequest.withOutput(power * DRUM_OUTPUT_SIGN);
    a1.setControl(request);
    a2.setControl(request);
    a3.setControl(request);
    a4.setControl(request);
  }

  public void spinDrum(double rpm) {
    targetRpm = Math.max(0.0, rpm);

    double measured = getRpm();
    double error = targetRpm - measured;
    boolean inTol = Math.abs(error) <= RPM_TOLERANCE;
    atGoal = atGoalDebounce.calculate(inTol);

    if (targetRpm <= 1e-3) {
      stop();
      return;
    }

    double targetRps = targetRpm / 60.0;
  var request = velocityRequest
    .withVelocity(targetRps * DRUM_OUTPUT_SIGN)
    .withAcceleration(MAX_ACCEL_RPS2);
    a1.setControl(request);
    a2.setControl(request);
    a3.setControl(request);
    a4.setControl(request);
  }

  public void stop() {
    targetRpm = 0.0;
    atGoal = false;
    var neutral = new NeutralOut();
    a1.setControl(neutral);
    a2.setControl(neutral);
    a3.setControl(neutral);
    a4.setControl(neutral);
  }


  /**
   * Refreshes all four velocity signals as one synchronized set and caches the result. Call once
   * per loop before reading {@link #getRpm()} or {@link #getMotorRpm(int)}.
   */
  public void refreshVelocities() {
    try {
      BaseStatusSignal.refreshAll(velocityRefresh);
    } catch (Exception ex) {
      // Fall through to whatever each signal last held.
    }
    double total = 0.0;
    for (int i = 0; i < motors.length; i++) {
      motorRpm[i] = Math.abs(safeSignalValue(velocitySignals[i])) * 60.0;
      total += motorRpm[i];
    }
    rawRpm = total / motorRpm.length;
    filteredRpm = rpmFilter.calculate(rawRpm);
  }

  /** Filtered drum speed. Use this for control and at-goal decisions. */
  public double getRpm() {
    return filteredRpm;
  }

  /** Unfiltered mean across the four rotors. Diagnostics only. */
  public double getRawRpm() {
    return rawRpm;
  }

  public double getMotorRpm(int index) {
    return motorRpm[index];
  }

  public int getMotorCount() {
    return motors.length;
  }

  private static double safeSignalValue(StatusSignal<?> signal) {
    try {
      return signal.getValueAsDouble();
    } catch (Exception ex) {
      return 0.0;
    }
  }

  public boolean isAtGoal() {
    return atGoal;
  }

  public void periodicTelemetry() {
    refreshVelocities();
    SmartDashboard.putNumber("Drum/ActualRPM", getRpm());
    SmartDashboard.putNumber("Drum/RawRPM", rawRpm);
    SmartDashboard.putNumber("Drum/FilterDeltaRPM", rawRpm - filteredRpm);
    SmartDashboard.putNumber("Drum/TargetRPM", targetRpm);
    SmartDashboard.putBoolean("Drum/AtGoal", atGoal);
    SmartDashboard.putNumber("Drum/ErrorRPM", targetRpm - getRpm());

    publishPerMotorTelemetry();

    double newKP = SmartDashboard.getNumber("Drum/Tuning/kP", liveKP);
    double newKV = SmartDashboard.getNumber("Drum/Tuning/kV", liveKV);
    double newKS = SmartDashboard.getNumber("Drum/Tuning/kS", liveKS);
    double newKA = SmartDashboard.getNumber("Drum/Tuning/kA", liveKA);

    if (newKP != liveKP || newKV != liveKV || newKS != liveKS || newKA != liveKA) {
      liveKP = newKP;
      liveKV = newKV;
      liveKS = newKS;
      liveKA = newKA;

      var newSlot0 = new Slot0Configs()
          .withKS(liveKS)
          .withKV(liveKV)
          .withKA(liveKA)
          .withKP(liveKP);
      a1.getConfigurator().apply(newSlot0);
      a2.getConfigurator().apply(newSlot0);
      a3.getConfigurator().apply(newSlot0);
      a4.getConfigurator().apply(newSlot0);
    }
  }

  /**
   * Per-motor breakout for tuning. getRpm() averages all four, so a dead or miswired motor just
   * drags the average down and reads as a gain problem -- RpmSpread surfaces that directly.
   * TorqueSaturated tells you the loop is clamped and gains are not what is shaping the response.
   */
  private void publishPerMotorTelemetry() {
    double minRpm = Double.POSITIVE_INFINITY;
    double maxRpm = Double.NEGATIVE_INFINITY;
    double totalSupply = 0.0;
    double maxAbsTorque = 0.0;

    for (int i = 0; i < motors.length; i++) {
      String label = "Drum/Motor/A" + (i + 1);
      double rpm = motorRpm[i];
      double torque = safeTorque(motors[i]);
      double supply = safeSupply(motors[i]);

      SmartDashboard.putNumber(label + "Rpm", rpm);
      SmartDashboard.putNumber(label + "TorqueCurrent", torque);
      SmartDashboard.putNumber(label + "SupplyCurrent", supply);

      minRpm = Math.min(minRpm, rpm);
      maxRpm = Math.max(maxRpm, rpm);
      totalSupply += supply;
      maxAbsTorque = Math.max(maxAbsTorque, Math.abs(torque));
    }

    SmartDashboard.putNumber("Drum/Motor/RpmSpread", maxRpm - minRpm);
    SmartDashboard.putNumber("Drum/SupplyCurrentTotal", totalSupply);
    SmartDashboard.putNumber("Drum/MaxTorqueCurrent", maxAbsTorque);
    SmartDashboard.putBoolean("Drum/TorqueSaturated", maxAbsTorque >= TORQUE_CURRENT_LIMIT - 2.0);
  }

  private static double safeTorque(TalonFX fx) {
    try {
      return fx.getTorqueCurrent().getValueAsDouble();
    } catch (Exception ex) {
      return 0.0;
    }
  }

  private static double safeSupply(TalonFX fx) {
    try {
      return fx.getSupplyCurrent().getValueAsDouble();
    } catch (Exception ex) {
      return 0.0;
    }
  }
}
