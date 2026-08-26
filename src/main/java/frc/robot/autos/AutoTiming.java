package frc.robot.autos;

import dev.doglog.DogLog;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

public class AutoTiming {
  private final String prefix;

  public AutoTiming(String prefix) {
    this.prefix = "Autos/Timing/" + prefix;
  }

  public Command time(String task, Command command) {
    var logName = prefix + "/" + task;

    return Commands.sequence(
            Commands.runOnce(() -> DogLog.time(logName)),
            command,
            Commands.runOnce(() -> DogLog.timeEnd(logName)))
        .withName("Timed" + command.getName());
  }

  public Command time(String task, Command... commands) {
    return time(task, Commands.sequence(commands));
  }
}
