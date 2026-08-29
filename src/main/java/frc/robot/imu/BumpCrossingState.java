package frc.robot.imu;

public enum BumpCrossingState {
  /** Not on a bump, and no crossing has been requested. Drive normally. */
  FLAT_NOT_CROSSING,
  /** A crossing was requested and we are driving at the bump, but have not tilted up yet. */
  FLAT_ABOUT_TO_CROSS,
  /** Tilted up along the crossing direction -- climbing the near face. */
  CROSSING_UPHILL,
  /** Tilted down along the crossing direction -- descending the far face. */
  CROSSING_DOWNHILL;

  /** True whenever the drive command should be overridden for a committed crossing. */
  public boolean isCrossing() {
    return this != FLAT_NOT_CROSSING;
  }
}
