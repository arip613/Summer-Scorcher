package frc.robot.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

/**
 * A CANrange time-of-flight sensor used as a beam break: something is "broken" when an object sits
 * closer than {@link #PROXIMITY_THRESHOLD_METERS}.
 *
 * <p>The threshold is evaluated on the device itself rather than in robot code, so the detection
 * edge is not subject to the 20 ms loop or CAN latency. The distance is still published so the
 * threshold can be checked against real readings.
 */
public class BeamBreak extends LifecycleSubsystem {
  /** Trip when an object is within this distance, in meters. */
  private static final double PROXIMITY_THRESHOLD_METERS = 0.20;

  /**
   * Deadband around the threshold, in meters. Without it an object sitting exactly at 0.20 m makes
   * the detected flag chatter as the reading jitters across the line.
   */
  private static final double PROXIMITY_HYSTERESIS_METERS = 0.02;

  /** Reject readings weaker than this; low signal means the reading is not trustworthy. */
  private static final double MIN_SIGNAL_STRENGTH = 2500.0;

  /** Filters single-sample dropouts so a brief glitch does not read as the object leaving. */
  private static final double DEBOUNCE_SECONDS = 0.04;

  private static final double UPDATE_HZ = 100.0;

  /**
   * A ball is present when the distance is under this, in meters.
   *
   * <p>An empty hopper reads about 1.0 m, the sensor ranging past the balls to whatever is behind
   * them; a ball intercepts the beam closer than that. So the threshold belongs just under the
   * empty baseline -- anything nearer than "empty" is a ball. 0.75 sat 0.25 m below it, which is
   * enough margin that a ball not fully seated read as an empty hopper and started the agitation
   * with balls still stacked.
   */
  private static final String BALL_THRESHOLD_KEY = "BeamBreak/Tune/BallThresholdMeters";
  private static final double DEFAULT_BALL_THRESHOLD_METERS = 0.85;

  /**
   * Falling-edge debounce on the ball reading. Balls bounce, so a settling stack opens brief gaps
   * in front of the sensor that are not the hopper running out. Falling only: a ball reappearing
   * registers immediately, but it takes this long of continuous absence to call the hopper empty.
   *
   * <p>This matters because the flag drives the intake arm. Entering PULSE resets the agitation to
   * stage 0, so a chattering reading would restart the sequence every few frames and leave the arm
   * twitching at the first position instead of working through the cycle.
   */
  private static final double BALL_DEBOUNCE_SECONDS = 0.15;

  private final CANrange sensor;
  private final StatusSignal<Boolean> detectedSignal;
  private final StatusSignal<org.wpilib.units.measure.Distance> distanceSignal;
  private final StatusSignal<Double> signalStrengthSignal;
  private final BaseStatusSignal[] allSignals;

  private final Debouncer detectedDebouncer =
      new Debouncer(DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);
  private final Debouncer hasBallDebouncer =
      new Debouncer(BALL_DEBOUNCE_SECONDS, Debouncer.DebounceType.kFalling);

  private boolean detected = false;
  private boolean detectedRaw = false;
  private double distanceMeters = 0.0;
  private double signalStrength = 0.0;
  private boolean hasBall = false;
  private boolean hasBallRaw = false;

  public BeamBreak(CANrange sensor) {
    super(SubsystemPriority.IMU);
    this.sensor = sensor;

    detectedSignal = sensor.getIsDetected();
    distanceSignal = sensor.getDistance();
    signalStrengthSignal = sensor.getSignalStrength();
    allSignals = new BaseStatusSignal[] {detectedSignal, distanceSignal, signalStrengthSignal};

    // Config and update-rate calls also talk to the device, so an absent sensor can throw here
    // too -- that would abort the Robot constructor before any subsystem is built.
    try {
      var cfg = new CANrangeConfiguration();
      cfg.ProximityParams = new ProximityParamsConfigs()
          .withProximityThreshold(PROXIMITY_THRESHOLD_METERS)
          .withProximityHysteresis(PROXIMITY_HYSTERESIS_METERS)
          .withMinSignalStrengthForValidMeasurement(MIN_SIGNAL_STRENGTH);
      sensor.getConfigurator().apply(cfg);

      for (var signal : allSignals) {
        signal.setUpdateFrequency(UPDATE_HZ);
      }
    } catch (Exception ex) {
      System.out.println("[BeamBreak] CANrange setup failed, sensor will read as absent: " + ex);
    }

    // setDefaultNumber, not putNumber: putNumber overwrites the tuned value on every boot, so a
    // threshold dialled in on the dashboard silently reverted on the next redeploy.
    SmartDashboard.setDefaultNumber(BALL_THRESHOLD_KEY, DEFAULT_BALL_THRESHOLD_METERS);
  }

  /**
   * True while a ball is in front of the sensor. Debounced on the falling edge, so a bouncing ball
   * momentarily clearing the sensor does not read as the hopper running dry.
   */
  public boolean hasBall() {
    return hasBall;
  }

  /** Undebounced ball reading, for checking the threshold against live distances. */
  public boolean hasBallRaw() {
    return hasBallRaw;
  }

  /** True while an object is within the threshold. Debounced. */
  public boolean isDetected() {
    return detected;
  }


  /** Measured distance in meters. 0 when the reading is invalid. */
  public double getDistanceMeters() {
    return distanceMeters;
  }

  @Override
  public void robotPeriodic() {
    // Every read below is guarded. A CANrange that is absent, on the wrong bus, or set to a
    // different device ID has never received data, so getValue() returns null -- unboxing that
    // into a boolean or calling .in() on it throws out of robotPeriodic, and neither
    // CommandScheduler.run() nor loopFunc() catches it, which takes the whole program down.
    // A missing sensor must degrade to "reads nothing", not kill the robot.
    boolean ok;
    try {
      ok = BaseStatusSignal.refreshAll(allSignals).isOK();
    } catch (Exception ex) {
      ok = false;
    }

    detectedRaw = readBoolean(detectedSignal);
    distanceMeters = readDistanceMeters(distanceSignal);
    signalStrength = readDouble(signalStrengthSignal);
    detected = detectedDebouncer.calculate(detectedRaw);

    // A dead sensor reads 0.0 m, which would otherwise look like a ball pressed right against it,
    // so require a live reading above zero.
    double threshold = SmartDashboard.getNumber(BALL_THRESHOLD_KEY, DEFAULT_BALL_THRESHOLD_METERS);
    hasBallRaw = ok && distanceMeters > 0.0 && distanceMeters < threshold;
    hasBall = hasBallDebouncer.calculate(hasBallRaw);

    SmartDashboard.putBoolean("BeamBreak/HasBall", hasBall);
    SmartDashboard.putBoolean("BeamBreak/HasBallRaw", hasBallRaw);
    SmartDashboard.putBoolean("BeamBreak/Detected", detected);
    SmartDashboard.putBoolean("BeamBreak/DetectedRaw", detectedRaw);
    SmartDashboard.putNumber("BeamBreak/DistanceMeters", distanceMeters);
    SmartDashboard.putNumber("BeamBreak/DistanceInches", distanceMeters * 39.3701);
    SmartDashboard.putNumber("BeamBreak/SignalStrength", signalStrength);
    SmartDashboard.putNumber("BeamBreak/ThresholdMeters", PROXIMITY_THRESHOLD_METERS);
    // OK here means the device is on the bus and answering. If this is false every other value
    // above is stale, which otherwise looks identical to "nothing in front of the sensor".
    SmartDashboard.putBoolean("BeamBreak/SensorOk", ok);
  }

  private static boolean readBoolean(StatusSignal<Boolean> signal) {
    try {
      Boolean value = signal.getValue();
      return value != null && value;
    } catch (Exception ex) {
      return false;
    }
  }

  private static double readDouble(StatusSignal<Double> signal) {
    try {
      Double value = signal.getValue();
      return value == null ? 0.0 : value;
    } catch (Exception ex) {
      return 0.0;
    }
  }

  private static double readDistanceMeters(
      StatusSignal<org.wpilib.units.measure.Distance> signal) {
    try {
      var value = signal.getValue();
      return value == null ? 0.0 : value.in(org.wpilib.units.Units.Meters);
    } catch (Exception ex) {
      return 0.0;
    }
  }
}
