package frc.robot.FlywheelSubsystem;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.RobotController;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.Timer;

/**
 * Dashboard-driven flywheel characterization.
 *
 * <p>Two routines, both run with {@link DrumStateMachine} parked in TUNING so nothing else commands
 * the motors:
 *
 * <ul>
 *   <li><b>Find kS</b> ramps open-loop torque current from zero until the drum breaks free, and
 *       reports the current it moved at.
 *   <li><b>Step Test</b> commands a velocity step, waits for it to settle, then averages the steady
 *       state and back-solves kV from the torque current actually needed to hold speed.
 * </ul>
 *
 * <p>All gains here are in amps, because the drum runs VelocityTorqueCurrentFOC.
 */
public class DrumTuner {
  private static final String KEY = "Drum/Tune/";

  // Find-kS sweep. At 3 A/s the full sweep to the ceiling takes ~13s, which is short enough to
  // hold through without reaching for disable. Let it run to completion -- it stops itself on
  // motion or at the ceiling.
  private static final double KS_RAMP_AMPS_PER_SEC = 3.0;
  /** Confirms the drum is really turning, and stops the sweep. */
  private static final double KS_MOTION_RPM = 20.0;
  /**
   * kS is reported at FIRST motion, not at KS_MOTION_RPM. The drum takes time to accelerate up to
   * the confirmation threshold and the ramp keeps climbing meanwhile, so reporting the confirming
   * current overestimates kS by however much it climbed during the spin-up.
   */
  private static final double KS_FIRST_MOTION_RPM = 3.0;
  private static final double KS_MAX_AMPS = 40.0;

  // Step test. Default sits mid-range of LookupTable's shot points (2200-2830 RPM), because a
  // single kV only fits drag well near the speed it was measured at.
  private static final double DEFAULT_STEP_RPM = 2500.0;
  // 40 RPM was too tight: a feedforward that is off by more than kP can pull back leaves the drum
  // parked just outside the band forever, and the run yields nothing.
  private static final double SETTLE_BAND_RPM = 75.0;
  private static final double SETTLE_HOLD_SEC = 0.5;
  private static final double SAMPLE_SEC = 1.0;
  private static final double STEP_TIMEOUT_SEC = 15.0;

  private final Drum drum;
  private final DrumStateMachine drumSM;
  private final Timer timer = new Timer();

  /** Set at init when the robot is disabled, so the routine bails instead of silently no-opping. */
  private boolean abortedDisabled;

  /** Commanded amps the instant the drum first budged. -1 until it does. */
  private double ksFirstMotionAmps = -1.0;

  // Step-test accumulators.
  private double peakRpm;
  private double settledAtSec;
  private double sampleStartSec;
  private int sampleCount;
  private double rpmSum;
  private double torqueSum;
  private double rpmMin;
  private double rpmMax;
  private double rawRpmMin;
  private double rawRpmMax;
  /** Lowest bus voltage across the whole step, including spin-up. */
  private double minBatteryVolts;
  /**
   * Lowest bus voltage during the steady-state sample window only. This is the one that matters
   * for kV: spin-up sag is expected and harmless, sag while holding speed is not.
   */
  private double minSampleVolts;

  public DrumTuner(Drum drum, DrumStateMachine drumSM) {
    this.drum = drum;
    this.drumSM = drumSM;

    SmartDashboard.putNumber(KEY + "StepRPM", DEFAULT_STEP_RPM);
    SmartDashboard.putData(KEY + "FindKs", findKsCommand());
    SmartDashboard.putData(KEY + "StepTest", stepTestCommand());
    SmartDashboard.putData(KEY + "Stop", stopCommand());
    SmartDashboard.putString(KEY + "Status", "idle");
  }

  /**
   * These routines are published with ignoringDisable so that clicking them while disabled still
   * runs long enough to say so. Without it the scheduler cancels them on the spot and the button
   * appears to do nothing at all. Motor output is still inert while disabled.
   */
  private boolean checkDisabled() {
    abortedDisabled = RobotState.isDisabled();
    if (abortedDisabled) {
      SmartDashboard.putString(
          KEY + "Status", "robot is DISABLED -- enable in Teleop, then run this again");
    }
    return abortedDisabled;
  }

  private Command stopCommand() {
    return Commands.runOnce(
            () -> {
              drum.stop();
              drumSM.requestOff();
              SmartDashboard.putString(KEY + "Status", "stopped");
            })
        .withName("DrumTune/Stop");
  }

  /**
   * Ramps torque current up slowly until the drum starts turning. The current at first motion is
   * kS. Runs open loop -- kP/kV/kA are not involved, so this is valid regardless of their values.
   */
  private Command findKsCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (checkDisabled()) {
                    return;
                  }
                  drumSM.requestTuning();
                  ksFirstMotionAmps = -1.0;
                  timer.restart();
                  SmartDashboard.putString(KEY + "Status", "finding kS...");
                }),
            Commands.run(
                    () -> {
                      double amps = timer.get() * KS_RAMP_AMPS_PER_SEC;
                      drum.setTorqueCurrent(amps);
                      if (ksFirstMotionAmps < 0.0 && drum.getRpm() >= KS_FIRST_MOTION_RPM) {
                        ksFirstMotionAmps = amps;
                        SmartDashboard.putNumber(KEY + "KsFirstMotionAmps", amps);
                      }
                      SmartDashboard.putNumber(KEY + "KsSweepAmps", amps);
                      SmartDashboard.putNumber(KEY + "KsSweepRPM", drum.getRpm());
                      // Measured vs commanded: if this stays near zero while KsSweepAmps climbs,
                      // the request is not reaching the motors and it is not a friction problem.
                      SmartDashboard.putNumber(
                          KEY + "KsSweepMeasuredAmps", drum.getAvgTorqueCurrent());
                      SmartDashboard.putNumber(
                          KEY + "KsSweepSupplyAmps", drum.getSupplyCurrentTotal());
                    })
                .until(
                    () ->
                        abortedDisabled
                            || drum.getRpm() >= KS_MOTION_RPM
                            || timer.get() * KS_RAMP_AMPS_PER_SEC >= KS_MAX_AMPS))
        .finallyDo(
            interrupted -> {
              if (abortedDisabled) {
                return;
              }
              double amps = timer.get() * KS_RAMP_AMPS_PER_SEC;
              drum.stop();
              drumSM.requestOff();
              boolean moved = ksFirstMotionAmps >= 0.0;
              SmartDashboard.putNumber(KEY + "FoundKs", moved ? ksFirstMotionAmps : -1.0);
              SmartDashboard.putNumber(KEY + "KsConfirmAmps", amps);
              SmartDashboard.putString(
                  KEY + "Status",
                  interrupted
                      ? "kS sweep interrupted"
                      : moved
                          ? String.format(
                              "kS = %.2f A at first motion (%.2f A at %.0f RPM)"
                                  + " -- copy into Drum/Tuning/kS",
                              ksFirstMotionAmps, amps, KS_MOTION_RPM)
                          : String.format("no motion by %.0f A -- check wiring", KS_MAX_AMPS));
            })
        .ignoringDisable(true)
        .withName("DrumTune/FindKs");
  }

  /**
   * Steps to StepRPM, waits for settle, then averages a window of steady state. kV is back-solved
   * from the current actually required to hold speed:
   *
   * <pre>kV = (steady per-motor torque current - kS) / (RPM / 60)</pre>
   */
  private Command stepTestCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (checkDisabled()) {
                    return;
                  }
                  drumSM.requestTuning();
                  drum.stop();
                  resetStepAccumulators();
                  timer.restart();
                  SmartDashboard.putString(KEY + "Status", "step test running...");
                }),
            Commands.run(this::stepTestPeriodic).until(this::stepTestDone))
        .finallyDo(
            interrupted -> {
              if (!abortedDisabled) {
                finishStepTest(interrupted);
              }
            })
        .ignoringDisable(true)
        .withName("DrumTune/StepTest");
  }

  private void resetStepAccumulators() {
    peakRpm = 0.0;
    settledAtSec = -1.0;
    sampleStartSec = -1.0;
    sampleCount = 0;
    rpmSum = 0.0;
    torqueSum = 0.0;
    rpmMin = Double.POSITIVE_INFINITY;
    rpmMax = Double.NEGATIVE_INFINITY;
    rawRpmMin = Double.POSITIVE_INFINITY;
    rawRpmMax = Double.NEGATIVE_INFINITY;
    minBatteryVolts = Double.POSITIVE_INFINITY;
    minSampleVolts = Double.POSITIVE_INFINITY;
  }

  private double stepTarget() {
    return Math.max(0.0, SmartDashboard.getNumber(KEY + "StepRPM", DEFAULT_STEP_RPM));
  }

  private void stepTestPeriodic() {
    double target = stepTarget();
    drum.spinDrum(target);

    double rpm = drum.getRpm();
    double now = timer.get();
    peakRpm = Math.max(peakRpm, rpm);

    // Tracked across the whole step, not just the sample window, because the worst sag happens
    // during spin-up. If the bus collapses the current loop cannot hold its command and the run
    // measures the battery rather than the flywheel.
    double volts = RobotController.getBatteryVoltage();
    minBatteryVolts = Math.min(minBatteryVolts, volts);
    SmartDashboard.putNumber(KEY + "BatteryVolts", volts);

    boolean inBand = Math.abs(target - rpm) <= SETTLE_BAND_RPM;
    if (!inBand) {
      // Fell back out of the band before the hold completed; require a fresh settle.
      if (sampleStartSec < 0.0) {
        settledAtSec = -1.0;
      }
      return;
    }

    if (settledAtSec < 0.0) {
      settledAtSec = now;
    }
    if (sampleStartSec < 0.0 && now - settledAtSec >= SETTLE_HOLD_SEC) {
      sampleStartSec = now;
    }
    if (sampleStartSec >= 0.0) {
      sampleCount++;
      rpmSum += rpm;
      torqueSum += drum.getAvgTorqueCurrent();
      rpmMin = Math.min(rpmMin, rpm);
      rpmMax = Math.max(rpmMax, rpm);
      double raw = drum.getRawRpm();
      rawRpmMin = Math.min(rawRpmMin, raw);
      rawRpmMax = Math.max(rawRpmMax, raw);
      minSampleVolts = Math.min(minSampleVolts, volts);
    }
  }

  private boolean stepTestDone() {
    if (abortedDisabled || timer.get() >= STEP_TIMEOUT_SEC) {
      return true;
    }
    return sampleStartSec >= 0.0 && timer.get() - sampleStartSec >= SAMPLE_SEC;
  }

  private void finishStepTest(boolean interrupted) {
    double target = stepTarget();
    // Read before stopping -- the drum is still at speed at this instant.
    double finalRpm = drum.getRpm();
    double finalTorque = drum.getAvgTorqueCurrent();
    drum.stop();
    drumSM.requestOff();

    // Published unconditionally: a run that never settles is still the most informative thing we
    // have, and bailing without these left nothing at all to diagnose from.
    SmartDashboard.putNumber(KEY + "PeakRPM", peakRpm);
    SmartDashboard.putNumber(KEY + "FinalRPM", finalRpm);
    SmartDashboard.putNumber(KEY + "FinalErrorRPM", target - finalRpm);
    SmartDashboard.putNumber(KEY + "FinalTorqueAmps", finalTorque);
    // Only sag while holding speed invalidates kV. Spin-up sag is expected and does not bias the
    // steady-state current measurement, since FOC torque-per-amp is bus-voltage independent.
    boolean sagged = Double.isFinite(minSampleVolts) && minSampleVolts < 10.5;
    SmartDashboard.putNumber(
        KEY + "MinBatteryVolts", Double.isFinite(minBatteryVolts) ? minBatteryVolts : -1.0);
    SmartDashboard.putNumber(
        KEY + "MinSampleVolts", Double.isFinite(minSampleVolts) ? minSampleVolts : -1.0);
    SmartDashboard.putBoolean(KEY + "BatterySagWarning", sagged);

    if (sampleCount == 0) {
      SmartDashboard.putString(
          KEY + "Status",
          interrupted
              ? String.format("interrupted -- reached %.0f of %.0f RPM", finalRpm, target)
              : String.format(
                  "never settled within %.0f RPM of %.0f -- peaked %.0f, ended %.0f (%.1f A)",
                  SETTLE_BAND_RPM, target, peakRpm, finalRpm, finalTorque));
      return;
    }

    double steadyRpm = rpmSum / sampleCount;
    double steadyTorque = torqueSum / sampleCount;
    double ripple = rpmMax - rpmMin;
    double kS = SmartDashboard.getNumber("Drum/Tuning/kS", Drum.kS);
    double targetRps = target / 60.0;
    double suggestedKv = targetRps > 1e-6 ? (steadyTorque - kS) / targetRps : 0.0;

    SmartDashboard.putNumber(KEY + "TimeToSettleS", settledAtSec);
    SmartDashboard.putNumber(KEY + "OvershootRPM", peakRpm - target);
    SmartDashboard.putNumber(KEY + "SteadyRPM", steadyRpm);
    SmartDashboard.putNumber(KEY + "SteadyErrorRPM", target - steadyRpm);
    SmartDashboard.putNumber(KEY + "RippleRPM", ripple);
    // Raw ripple is the sensor scatter; filtered ripple is what at-goal actually sees.
    SmartDashboard.putNumber(KEY + "RippleRawRPM", rawRpmMax - rawRpmMin);
    SmartDashboard.putNumber(KEY + "SteadyTorqueAmps", steadyTorque);
    SmartDashboard.putNumber(KEY + "SuggestedKv", suggestedKv);
    SmartDashboard.putString(
        KEY + "Status",
        interrupted
            ? "step test interrupted (partial results)"
            : String.format(
                "settled %.2fs, ripple %.0f RPM, suggested kV = %.4f%s",
                settledAtSec,
                ripple,
                suggestedKv,
                sagged
                    ? String.format(
                        " -- BATTERY SAGGED to %.1fV while holding speed, kV suspect",
                        minSampleVolts)
                    : ""));
  }
}
