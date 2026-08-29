package frc.robot.fms;

import dev.doglog.DogLog;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.DataLogManager;

public class FmsSubsystem extends LifecycleSubsystem {
  private String lastMetadata = "";
  private boolean logRenamedForMatch = false;

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

  /**
   * Identifier for the current match, e.g. {@code 2026week0_Q42}. Empty when FMS has not reported
   * a match yet.
   */
  public static String getMatchName() {
    var type = MatchState.getMatchType();
    if (type == MatchType.NONE) {
      return "";
    }

    String prefix =
        switch (type) {
          case PRACTICE -> "P";
          case QUALIFICATION -> "Q";
          case ELIMINATION -> "E";
          case NONE -> "";
        };

    String event = sanitize(MatchState.getEventName());
    String name = prefix + MatchState.getMatchNumber();
    int replay = MatchState.getReplayNumber();
    if (replay > 0) {
      name += "r" + replay;
    }

    return event.isEmpty() ? name : event + "_" + name;
  }

  /** Strips anything that has no business in a filename. Event names come from the field. */
  private static String sanitize(String raw) {
    if (raw == null) {
      return "";
    }
    return raw.trim().replaceAll("[^A-Za-z0-9._-]", "");
  }

  @Override
  public void robotPeriodic() {
    DogLog.log("Fms/Alliance", isRedAlliance() ? "Red" : "Blue");

    // Match time is the clock every other log entry gets read against -- without it a dump can
    // show what happened but not when in the match it happened. Counts down within each period,
    // and is -1 when there is no match running.
    DogLog.log("Match/TimeRemaining", MatchState.getMatchTime());
    DogLog.log("Match/FmsAttached", RobotState.isFMSAttached());
    DogLog.log("Match/Enabled", RobotState.isEnabled());

    logMetadataOnChange();
    renameLogForMatch();
  }

  /**
   * Match identity, written only when it changes. These are strings and would otherwise churn the
   * log at 50 Hz for values that change a few times a day.
   */
  private void logMetadataOnChange() {
    String event = MatchState.getEventName();
    var type = MatchState.getMatchType();
    int number = MatchState.getMatchNumber();
    int replay = MatchState.getReplayNumber();
    int station = MatchState.getLocation().orElse(-1);

    String signature = event + "|" + type + "|" + number + "|" + replay + "|" + station;
    if (signature.equals(lastMetadata)) {
      return;
    }
    lastMetadata = signature;

    DogLog.log("Match/EventName", event == null ? "" : event);
    DogLog.log("Match/Type", type.toString());
    DogLog.log("Match/Number", number);
    DogLog.log("Match/ReplayNumber", replay);
    DogLog.log("Match/DriverStation", station);
    DogLog.log("Match/Name", getMatchName());
  }

  /**
   * Re-opens the data log under the match name once FMS reports one.
   *
   * <p>The log file is created at boot, long before the robot is on the field, and this WPILib
   * alpha has no way to rename an open log -- DataLog exposes flush/pause/resume and nothing else.
   * So the log is stopped and restarted under the match name instead. Everything from the pits
   * stays in the boot-named file and the match gets its own, which is the split you want anyway.
   *
   * <p>Only ever done while disabled. FMS connects during the pre-match, so there is no reason to
   * take the risk mid-match, and a failed restart falls back to a default-named log rather than no
   * log at all.
   */
  private void renameLogForMatch() {
    if (logRenamedForMatch || RobotBase.isSimulation()) {
      return;
    }
    if (!RobotState.isFMSAttached() || RobotState.isEnabled()) {
      return;
    }

    String matchName = getMatchName();
    if (matchName.isEmpty()) {
      return;
    }

    // Latch before attempting, so a throwing restart cannot retry every loop for a whole match.
    logRenamedForMatch = true;

    // Must be read before stop(), which clears it.
    String logDir = DataLogManager.getLogDir();

    try {
      DataLogManager.stop();
      DataLogManager.start(logDir, matchName + ".wpilog");
      // DogLog writes through NetworkTables and DataLogManager re-attaches NT capture on start,
      // so DogLog entries follow the new file on their own. Driver Station capture does not --
      // it was bound to the old DataLog instance and has to be pointed at the new one.
      DriverStation.startDataLog(DataLogManager.getLog());

      DogLog.log("Match/LogRenamedTo", matchName + ".wpilog");
      // Metadata lives in the old file; force it into the new one.
      lastMetadata = "";
    } catch (Exception ex) {
      System.out.println("[Fms] Could not reopen log as " + matchName + ": " + ex);
      try {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
      } catch (Exception fallback) {
        System.out.println("[Fms] Fallback log restart also failed: " + fallback);
      }
    }
  }
}
