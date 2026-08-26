package frc.robot.util.scheduling;

import org.wpilib.driverstation.RobotState;
import org.wpilib.command2.CommandScheduler;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public final class LifecycleSubsystemManager {

  public static LifecycleStage getStage() {
    if (RobotState.isTeleopEnabled()) {
      return LifecycleStage.TELEOP;
    } else if (RobotState.isAutonomousEnabled()) {
      return LifecycleStage.AUTONOMOUS;
    } else if (RobotState.isUtilityEnabled()) {
      return LifecycleStage.TEST;
    } else {

      return LifecycleStage.DISABLED;
    }
  }

  private static final List<LifecycleSubsystem> subsystems = new ArrayList<>();
  private static final CommandScheduler commandScheduler = CommandScheduler.getInstance();

  public static void ready() {
    subsystems.sort(
        Comparator.comparingInt((LifecycleSubsystem subsystem) -> subsystem.priority.value)
            .reversed());

    for (LifecycleSubsystem lifecycleSubsystem : subsystems) {
      commandScheduler.registerSubsystem(lifecycleSubsystem);
    }
  }



  static void registerSubsystem(LifecycleSubsystem subsystem) {
    subsystems.add(subsystem);
    commandScheduler.unregisterSubsystem(subsystem);
  }

  private LifecycleSubsystemManager() {}
}
