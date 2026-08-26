package frc.robot.fms;

import dev.doglog.DogLog;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.framework.RobotBase;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class FmsSubsystem extends LifecycleSubsystem {
  public FmsSubsystem() {
    super(SubsystemPriority.FMS);
  }

  public static boolean isRedAlliance() {
    // Desktop simulation starts at the blue pose, so an unset simulation alliance must also be
    // blue. Preserve the existing red fallback on the real robot when FMS is unavailable.
    Alliance defaultAlliance = RobotBase.isSimulation() ? Alliance.BLUE : Alliance.RED;
    Alliance alliance = MatchState.getAlliance().orElse(defaultAlliance);

    return alliance == Alliance.RED;
  }

  @Override
  public void robotPeriodic() {
    DogLog.log("Fms/Alliance", isRedAlliance() ? "Red" : "Blue");
  }
}
