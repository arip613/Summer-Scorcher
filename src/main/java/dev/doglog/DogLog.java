package dev.doglog;

import org.wpilib.math.geometry.Pose2d;

/**
 * Minimal stub of DogLog for development builds.
 * All methods are no-ops to satisfy compile-time dependencies.
 */
public final class DogLog {
  public static void logFault(String message) {
    // no-op
  }

  public static void time(String name) {
    // no-op
  }

  public static void timeEnd(String name) {
    // no-op
  }

  public static void log(String name, Pose2d[] poses) {
    // no-op
  }

  public static void log(String name, Object value) {
    // no-op
  }

  private DogLog() {}

public static void tunable(String string, double kI, Object object) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'tunable'");
}
}
