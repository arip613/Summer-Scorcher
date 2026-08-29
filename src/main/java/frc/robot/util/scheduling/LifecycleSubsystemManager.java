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
  private static boolean ready = false;

  public static void ready() {
    ready = true;
    subsystems.sort(
        Comparator.comparingInt((LifecycleSubsystem subsystem) -> subsystem.priority.value)
            .reversed());

    for (LifecycleSubsystem lifecycleSubsystem : subsystems) {
      commandScheduler.registerSubsystem(lifecycleSubsystem);
    }
  }



  static void registerSubsystem(LifecycleSubsystem subsystem) {
    subsystems.add(subsystem);

    if (ready) {
      // Constructed after ready() already ran. Unregistering here would leave the subsystem with
      // no periodic at all, and nothing would say so -- BumpCrossingTracker was built this way and
      // its state machine simply never ticked, so a crossing armed and never finished, holding an
      // open loop drive override for the rest of auto. Register it immediately instead, and say
      // something, because the ordering is still worth fixing at the source.
      System.out.println(
          "[LifecycleSubsystemManager] "
              + subsystem.getClass().getSimpleName()
              + " was constructed after ready(); registering it directly. Construct it before"
              + " ready() so it runs in priority order.");
      commandScheduler.registerSubsystem(subsystem);
      return;
    }

    commandScheduler.unregisterSubsystem(subsystem);
  }

  private LifecycleSubsystemManager() {}
}
