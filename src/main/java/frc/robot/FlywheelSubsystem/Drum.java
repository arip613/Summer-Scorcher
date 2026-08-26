package frc.robot.FlywheelSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.smartdashboard.SmartDashboard;

public class Drum {
  private final TalonFX a1, a2, a3, a4;

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  public static final double kS = 0.3;
  public static final double kV = 0.225;
  public static final double kP = 11;
  public static final double MAX_ACCEL_RPS2 = 350;

  public static final double SUPPLY_LIMIT = 60; //60
  public static final double STATOR_LIMIT = 80; //80
  public static final double TORQUE_CURRENT_LIMIT = 80; //80

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

  public Drum(TalonFX a1, TalonFX a2, TalonFX a3, TalonFX a4) {
    this.a1 = a1;
    this.a2 = a2;
    this.a3 = a3;
    this.a4 = a4;

    var cfg = new TalonFXConfiguration();

  cfg.Slot0 = new Slot0Configs()
    .withKS(kS)
    .withKV(kV)
    .withKP(kP);

    cfg.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(SUPPLY_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(STATOR_LIMIT)
        .withStatorCurrentLimitEnable(false);

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


  public double getRpm() {
    double v1 = safeVel(a1);
    double v2 = safeVel(a2);
    double v3 = safeVel(a3);
    double v4 = safeVel(a4);
    double rpsAvg = (Math.abs(v1) + Math.abs(v2) + Math.abs(v3) + Math.abs(v4)) / 4.0;
    return rpsAvg * 60.0;
  }

  public boolean isAtGoal() {
    return atGoal;
  }

  private static double safeVel(TalonFX fx) {
    try {
      return fx.getRotorVelocity().getValueAsDouble();
    } catch (Exception ex) {
      return 0.0;
    }
  }

  public void periodicTelemetry() {
    SmartDashboard.putNumber("Drum/ActualRPM", getRpm());
    SmartDashboard.putNumber("Drum/TargetRPM", targetRpm);
    SmartDashboard.putBoolean("Drum/AtGoal", atGoal);
    SmartDashboard.putNumber("Drum/ErrorRPM", targetRpm - getRpm());

    double newKP = SmartDashboard.getNumber("Drum/Tuning/kP", liveKP);
    double newKV = SmartDashboard.getNumber("Drum/Tuning/kV", liveKV);
    double newKS = SmartDashboard.getNumber("Drum/Tuning/kS", liveKS);

    if (newKP != liveKP || newKV != liveKV || newKS != liveKS) {
      liveKP = newKP;
      liveKV = newKV;
      liveKS = newKS;

      var newSlot0 = new Slot0Configs()
          .withKS(liveKS)
          .withKV(liveKV)
          .withKP(liveKP);
      a1.getConfigurator().apply(newSlot0);
      a2.getConfigurator().apply(newSlot0);
      a3.getConfigurator().apply(newSlot0);
      a4.getConfigurator().apply(newSlot0);
    }
  }
}
