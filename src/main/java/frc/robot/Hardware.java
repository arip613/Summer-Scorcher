package frc.robot;

import org.wpilib.command2.button.CommandGamepad;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightModel;
import frc.robot.vision.limelight.LimelightState;

public class Hardware {

  // SystemCore has five built-in CAN buses (can_s0..can_s4). The roboRIO's "rio" bus and
  // CANivore USB adapters do not exist on this platform -- naming them made Phoenix fail
  // with "CANbus Failed to Connect". Bus assignments verified by candump on the robot:
  // can_s0 carries device IDs 0,1,2,3,5-12,21-29,51; can_s4 carries 31,32.
  private static final CANBus CANIVORE = new CANBus("can_s0");
  private static final CANBus RIO      = new CANBus("can_s4");

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
  public final TalonFX intakeRollerMotorA = new TalonFX(31, RIO);   
  public final TalonFX intakeRollerMotorB     = new TalonFX(32, RIO);

  public final Limelight leftLimelight  = new Limelight("left",  LimelightState.TAGS, LimelightModel.FOUR);
  public final Limelight rightLimelight = new Limelight("right", LimelightState.TAGS, LimelightModel.FOUR);

}
