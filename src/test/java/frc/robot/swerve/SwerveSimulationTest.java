package frc.robot.swerve;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import frc.robot.AutoMovements.DriveToPose;
import frc.robot.localization.LocalizationSubsystem;

class SwerveSimulationTest {
  @Test
  void drivetrainPhysicsUpdatesAndClosesHeadingLoop() throws InterruptedException {
    assertTrue(HAL.initialize(500, 0));
    assertTrue(Utils.isSimulation(), "Phoenix did not load its simulation backend");
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStationBackend.refreshData();
    assertTrue(RobotState.isEnabled());

    SwerveSubsystem swerve = new SwerveSubsystem();
    try {
      // First collection sets the simulated slow-mode multiplier to 1.0.
      swerve.robotPeriodic();
      // Simulation defaults to the blue alliance, so no alliance flip applies here. The field
      // frame is blue-origin: +X points at the red wall, +Y is the blue driver's left.
      //
      // getLeftX() is "right is positive" and getLeftY() is "back is positive", so a stick pushed
      // forward-left is (x=-1, y=-1). These previously asserted the opposite sign on both axes,
      // which matched the shipped behaviour but not the field frame -- on the real robot that
      // showed up as translation being inverted for both alliances.
      swerve.driveTeleop(0.0, -1.0, 0.0);
      assertTrue(
          swerve.getTeleopSpeeds().vx > 0.0,
          "stick forward must drive toward +X for a blue driver");

      swerve.driveTeleop(-1.0, 0.0, 0.0);
      assertTrue(
          swerve.getTeleopSpeeds().vy > 0.0,
          "stick left must drive toward +Y for a blue driver");

      double initialHeading = swerve.getSimPose().getRotation().getDegrees();
      Unmanaged.feedEnable(1000);
      swerve.driveTeleop(0.0, 0.0, 1.0);
      assertTrue(
          Math.abs(swerve.getTeleopSpeeds().omega) > 1.0,
          "right controller X did not produce a rotation command");

      Thread.sleep(750);

      double finalHeading = swerve.getSimPose().getRotation().getDegrees();
      assertTrue(
          Math.abs(finalHeading - initialHeading) > 5.0,
          "simulated heading did not respond to a rotation request");

      swerve.driveTeleop(0.0, 0.0, 0.0);
      swerve.snapsDriveRequest(90.0);
      Thread.sleep(1250);

      double snappedHeading = swerve.getSimPose().getRotation().getDegrees();
      assertTrue(
          Math.abs(frc.robot.AutoMovements.HeadingLockMath.errorDegrees(90.0, snappedHeading))
              < 5.0,
          "simulated heading controller did not converge to its target");

      DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
      DriverStationSim.notifyNewData();
      DriverStationBackend.refreshData();
      swerve.robotPeriodic();

      Pose2d autoTarget = new Pose2d(
          swerve.getSimPose().getX() + 1.0,
          swerve.getSimPose().getY(),
          Rotation2d.fromDegrees(90.0));
      LocalizationSubsystem localization = new LocalizationSubsystem(null, null, swerve);
      DriveToPose autoDrive = new DriveToPose(swerve, localization, () -> autoTarget, true, 2.0);
      autoDrive.initialize();
      long deadline = System.nanoTime() + 3_000_000_000L;
      while (!autoDrive.isFinished() && System.nanoTime() < deadline) {
        autoDrive.execute();
        swerve.robotPeriodic();
        Thread.sleep(20);
      }
      autoDrive.end(false);
      assertTrue(
          swerve.getSimPose().getTranslation().getDistance(autoTarget.getTranslation()) < 0.5,
          "autonomous DriveToPose fallback did not move the simulated robot to its waypoint");
    } finally {
      swerve.closeSimulation();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    }
  }
}
