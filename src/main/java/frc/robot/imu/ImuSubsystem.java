package frc.robot.imu;

import com.ctre.phoenix6.hardware.Pigeon2;
import dev.doglog.DogLog;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.util.function.DoubleSupplier;
import org.wpilib.framework.RobotBase;

public class ImuSubsystem extends StateMachine<ImuState> {
  private static final double IS_TILTED_THRESHOLD = 4.0;

  /**
   * Roll the Pigeon reports with the robot sitting flat. This one is mounted inverted, so level
   * reads about 180 rather than 0 -- measured at pitch 0.57, roll 179.21 on the floor, from match
   * log AZGLE4_Q3.
   *
   * <p>Anything reasoning about how tilted the robot is has to work from level-referenced values,
   * not the raw frame. Using raw roll made a level robot look 179 degrees tilted, which rejected
   * every vision measurement for an entire match.
   */
  private static final double MOUNT_ROLL_OFFSET_DEGREES = 180.0;
  private static final Debouncer IS_TILTED_DEBOUNCE = new Debouncer(0.5, DebounceType.kRising);
  private final Pigeon2 imu;
  private final DoubleSupplier simulatedHeadingDegrees;
  private final DoubleSupplier simulatedAngularVelocityDegreesPerSecond;
  private double robotHeading = 0;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;

  public ImuSubsystem(Pigeon2 imu) {
    this(imu, null, null);
  }

  public ImuSubsystem(
      Pigeon2 imu,
      DoubleSupplier simulatedHeadingDegrees,
      DoubleSupplier simulatedAngularVelocityDegreesPerSecond) {
    super(SubsystemPriority.IMU, ImuState.DEFAULT_STATE);
    this.imu = imu;
    this.simulatedHeadingDegrees = simulatedHeadingDegrees;
    this.simulatedAngularVelocityDegreesPerSecond = simulatedAngularVelocityDegreesPerSecond;
  }

  @Override
  protected void collectInputs() {
    if (RobotBase.isSimulation() && simulatedHeadingDegrees != null) {
      robotHeading = MathUtil.inputModulus(simulatedHeadingDegrees.getAsDouble(), -180, 180);
      angularVelocity = simulatedAngularVelocityDegreesPerSecond.getAsDouble();
      pitch = 0.0;
      pitchRate = 0.0;
      roll = 0.0;
      rollRate = 0.0;
      return;
    }
    robotHeading = MathUtil.inputModulus(imu.getYaw().getValueAsDouble(), -180, 180);
    angularVelocity = imu.getAngularVelocityZWorld().getValueAsDouble();
    pitch = imu.getPitch().getValueAsDouble();
    pitchRate = imu.getAngularVelocityYWorld().getValueAsDouble();
    roll = imu.getRoll().getValueAsDouble();
    rollRate = imu.getAngularVelocityXWorld().getValueAsDouble();
  }

  public double getRobotHeading() {
    return robotHeading;
  }

  public double getRobotAngularVelocity() {
    return angularVelocity;
  }

  public double getPitch() {
    return pitch;
  }

  public double getPitchRate() {
    return pitchRate;
  }

  public double getRoll() {
    return roll;
  }

  /**
   * Pitch relative to the robot sitting level. Same as {@link #getPitch()} today, paired with
   * {@link #getLevelRoll()} so callers do not have to know which axis carries the mount offset.
   */
  public double getLevelPitch() {
    return MathUtil.inputModulus(pitch, -180, 180);
  }

  /** Roll relative to the robot sitting level, with the inverted mount offset removed. */
  public double getLevelRoll() {
    return MathUtil.inputModulus(roll - MOUNT_ROLL_OFFSET_DEGREES, -180, 180);
  }

  public double getRollRate() {
    return rollRate;
  }

  public boolean isFlatDebounced() {
    // Level-referenced, for the same reason the vision tilt gate is: raw roll reads ~180 with the
    // robot flat on this mount, so comparing it against 0 reported "not flat" permanently. Unused
    // today, but it would have been wrong the moment anything started calling it.
    return IS_TILTED_DEBOUNCE.calculate(
        MathUtil.isNear(getLevelPitch(), 0, IS_TILTED_THRESHOLD)
            && MathUtil.isNear(getLevelRoll(), 0, IS_TILTED_THRESHOLD));
  }

  public void setAngle(double zeroAngle) {
    this.imu.setYaw(zeroAngle);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    // Raw and level-referenced both: a mismatch between them is exactly the failure that is
    // invisible without logging, since the raw values look perfectly reasonable in isolation.
    DogLog.log("Imu/Pitch", pitch);
    DogLog.log("Imu/Roll", roll);
    DogLog.log("Imu/LevelPitch", getLevelPitch());
    DogLog.log("Imu/LevelRoll", getLevelRoll());
  }
}
