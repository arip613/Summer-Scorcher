package frc.robot.util;
import java.util.function.BooleanSupplier;

public class FeatureFlag {
  public static BooleanSupplier of(String name, boolean defaultValue) {

    return () -> defaultValue;
  }

  private FeatureFlag() {}
}
