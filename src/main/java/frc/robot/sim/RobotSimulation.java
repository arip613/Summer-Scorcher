package frc.robot.sim;

import frc.robot.AutoMovements.FieldPoints;
import frc.robot.Hardware;
import frc.robot.swerve.SwerveSubsystem;
import java.util.List;
import org.wpilib.framework.RobotBase;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;

/** Robot-wide desktop simulation support. The drivetrain physics runs inside Phoenix. */
public final class RobotSimulation implements AutoCloseable {
  private static final double PERIOD_SECONDS = 0.005;
  private static final Pose2d BLUE_START_POSE =
      new Pose2d(2.0, 4.0, Rotation2d.kZero);
  private static final Pose2d RED_START_POSE = FieldPoints.mirrorPose(BLUE_START_POSE);

  private final SwerveSubsystem swerve;
  private final List<TalonFXMotorSim> mechanismMotors;
  private final StructPublisher<Pose2d> robotPose2dPublisher;
  private final StructPublisher<Pose3d> robotPose3dPublisher;
  private final StructPublisher<Pose3d> redHubPosePublisher;
  private final StructPublisher<Pose3d> blueHubPosePublisher;
  private final StructArrayPublisher<Pose3d> hubPosesPublisher;
  private final Notifier mechanismNotifier;
  private final Notifier batteryNotifier;
  private volatile double mechanismCurrentAmps;
  private volatile double totalCurrentAmps;
  private volatile double loadedBatteryVoltage = 12.0;
  private Alliance spawnAlliance;

  public RobotSimulation(Hardware hardware, SwerveSubsystem swerve) {
    if (!RobotBase.isSimulation()) {
      throw new IllegalStateException("RobotSimulation can only be created on desktop simulation");
    }
    this.swerve = swerve;

    var networkTables = NetworkTableInstance.getDefault();
    robotPose2dPublisher =
        networkTables.getStructTopic("/Simulation/Field/RobotPose2d", Pose2d.struct).publish();
    robotPose3dPublisher =
        networkTables.getStructTopic("/Simulation/Field/RobotPose3d", Pose3d.struct).publish();
    redHubPosePublisher =
        networkTables.getStructTopic("/Simulation/Field/RedHubPose", Pose3d.struct).publish();
    blueHubPosePublisher =
        networkTables.getStructTopic("/Simulation/Field/BlueHubPose", Pose3d.struct).publish();
    hubPosesPublisher =
        networkTables.getStructArrayTopic("/Simulation/Field/HubPoses", Pose3d.struct).publish();

    // Rotor-side inertias are intentionally approximate. They make closed-loop sensors and
    // current draw realistic enough for whole-robot testing without duplicating mechanism CAD.
    mechanismMotors = List.of(
        new TalonFXMotorSim(hardware.drumA1, 0.003, false),
        new TalonFXMotorSim(hardware.drumA2, 0.003, false),
        new TalonFXMotorSim(hardware.drumA3, 0.003, true),
        new TalonFXMotorSim(hardware.drumA4, 0.003, true),
        new TalonFXMotorSim(hardware.hopperMotor, 0.002, false),
        new TalonFXMotorSim(hardware.hoodMotor, 0.006, false),
        new TalonFXMotorSim(hardware.indexerMotor, 0.002, false),
        new TalonFXMotorSim(hardware.indexerMotor2, 0.002, false),
        new TalonFXMotorSim(hardware.intakePivotMotor, 0.006, false),
        new TalonFXMotorSim(hardware.intakeRollerMotorA, 0.002, false),
        new TalonFXMotorSim(hardware.intakeRollerMotorB, 0.002, true));

    mechanismNotifier = new Notifier(this::updateMechanisms);
    batteryNotifier = new Notifier(this::updateBattery);
  }

  public void initialize() {
    RoboRioSim.setTeamNumber(10183);
    RoboRioSim.setVInVoltage(12.0);
    spawnAlliance = MatchState.getAlliance().orElse(Alliance.BLUE);
    resetForAlliance(spawnAlliance);
    mechanismNotifier.startPeriodic(PERIOD_SECONDS);
    // HAL voltage callbacks can occasionally take longer than a 20 ms robot loop while Phoenix
    // starts. Battery dynamics do not need the 200 Hz mechanism rate, so update them separately.
    batteryNotifier.startPeriodic(0.1);

    SmartDashboard.putData(
        "Simulation/ResetBlueStart", swerve.runOnce(this::resetBlueStart).ignoringDisable(true));
    SmartDashboard.putData(
        "Simulation/ResetRedStart", swerve.runOnce(this::resetRedStart).ignoringDisable(true));
  }

  public void periodic() {
    // The simulation DS alliance can arrive after robot construction. Follow an explicit alliance
    // change while disabled so Red 1/2/3 and Blue 1/2/3 always get their matching start pose.
    MatchState.getAlliance().ifPresent(alliance -> {
      if (RobotState.isDisabled() && alliance != spawnAlliance) {
        spawnAlliance = alliance;
        resetForAlliance(alliance);
      }
    });

    Pose2d pose = swerve.getSimPose();
    Pose3d robotPose3d = new Pose3d(pose);
    Pose3d redHubPose = new Pose3d(
        new Pose2d(FieldPoints.getHeadingLockRedPoint(), Rotation2d.kZero));
    Pose3d blueHubPose = new Pose3d(
        new Pose2d(FieldPoints.getHeadingLockBluePoint(), Rotation2d.kZero));

    robotPose2dPublisher.set(pose);
    robotPose3dPublisher.set(robotPose3d);
    redHubPosePublisher.set(redHubPose);
    blueHubPosePublisher.set(blueHubPose);
    hubPosesPublisher.set(new Pose3d[] {redHubPose, blueHubPose});

    SmartDashboard.putNumberArray(
        "Simulation/RobotPose",
        new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
    SmartDashboard.putNumber("Simulation/TotalCurrentAmps", totalCurrentAmps);
    SmartDashboard.putNumber("Simulation/BatteryVoltage", loadedBatteryVoltage);
  }

  public void resetBlueStart() {
    swerve.resetPose(BLUE_START_POSE);
  }

  public void resetRedStart() {
    swerve.resetPose(RED_START_POSE);
  }

  private void resetForAlliance(Alliance alliance) {
    if (alliance == Alliance.RED) {
      resetRedStart();
    } else {
      resetBlueStart();
    }
  }

  private void updateMechanisms() {
    double supplyVoltage = RobotController.getBatteryVoltage();
    double current = 0.0;
    for (TalonFXMotorSim motor : mechanismMotors) {
      current += motor.update(PERIOD_SECONDS, supplyVoltage);
    }
    mechanismCurrentAmps = current;
  }

  private void updateBattery() {
    totalCurrentAmps = swerve.getSimEstimatedSupplyCurrentAmps() + mechanismCurrentAmps;
    loadedBatteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentAmps);
    RoboRioSim.setVInVoltage(loadedBatteryVoltage);
  }

  @Override
  public void close() {
    mechanismNotifier.close();
    batteryNotifier.close();
    robotPose2dPublisher.close();
    robotPose3dPublisher.close();
    redHubPosePublisher.close();
    blueHubPosePublisher.close();
    hubPosesPublisher.close();
  }
}
