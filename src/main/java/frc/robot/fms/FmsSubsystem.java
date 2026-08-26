package frc.robot.fms;

import dev.doglog.DogLog;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class FmsSubsystem extends LifecycleSubsystem {
  public FmsSubsystem() {
    super(SubsystemPriority.FMS);
  }

  public static boolean isRedAlliance() {
    // When not connected to FMS (off-field) DriverStation may return empty.
    // Default to Alliance.RED when FMS is unavailable.
    Alliance alliance = MatchState.getAlliance().orElse(Alliance.RED);

    return alliance == Alliance.RED;
  }

  @Override
  public void robotPeriodic() {
    DogLog.log("Fms/Alliance", isRedAlliance() ? "Red" : "Blue");
  }
}
