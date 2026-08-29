package frc.robot;

import org.wpilib.command2.button.CommandGamepad;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.config.CanBuses;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightModel;
import frc.robot.vision.limelight.LimelightState;

public class Hardware {

  // Buses are defined in CanBuses so the drivetrain and the mechanisms cannot end up naming
  // different ones. Everything that used to sit on SystemCore's can_s0 -- the swerve drivetrain
  // and every mechanism motor -- is now on the CANivore; can_s0 is unused. The intake rollers
  // stay on can_s4 and the CANrange on can_s1.
  private static final CANBus CANIVORE = CanBuses.CANIVORE;
  private static final CANBus ROLLER_BUS = CanBuses.ROLLERS;
  private static final CANBus SENSOR_BUS = CanBuses.SENSORS;

  public final CommandGamepad driverController = new CommandGamepad(0);
  public final CommandGamepad operatorController   = new CommandGamepad(1); 

  public final TalonFX drumA1            = new TalonFX(21, CANIVORE);
  public final TalonFX drumA2            = new TalonFX(22, CANIVORE);
  public final TalonFX drumA3            = new TalonFX(23, CANIVORE);
  public final TalonFX drumA4            = new TalonFX(24, CANIVORE);
  public final TalonFX hopperMotor       = new TalonFX(29, CANIVORE);
  public final TalonFX hoodMotor         = new TalonFX(27, CANIVORE);
  public final TalonFX indexerMotor      = new TalonFX(25, CANIVORE);
  public final TalonFX indexerMotor2     = new TalonFX(26, CANIVORE);
  public final TalonFX intakePivotMotor  = new TalonFX(28, CANIVORE);
  public final TalonFX intakeRollerMotorA = new TalonFX(31, ROLLER_BUS);   
  public final TalonFX intakeRollerMotorB     = new TalonFX(32, ROLLER_BUS);

  /** CANrange on can_s1, ID 0, used as a beam break. */
  public final CANrange beamBreakSensor = new CANrange(0, SENSOR_BUS);

  /**
   * Never commanded, never read -- constructed only to register a TalonFX on can_s1.
   *
   * <p>Phoenix brings a bus up lazily, and the diagnostic server has historically only enumerated
   * buses that have a motor controller on them. If the CANrange alone is not enough to start
   * can_s1, this forces the bus to initialize so Tuner can see what is actually on it. ID 62 is
   * outside every range in use, so it will not collide with a real device.
   */
  @SuppressWarnings("unused")
  private final TalonFX ghostSensorBusDevice = new TalonFX(62, SENSOR_BUS);

  public final Limelight leftLimelight  = new Limelight("left",  LimelightState.TAGS, LimelightModel.FOUR);
  public final Limelight rightLimelight = new Limelight("right", LimelightState.TAGS, LimelightModel.FOUR);

}
