package frc.robot.config;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;

/**
 * Every CAN bus on the robot, defined once.
 *
 * <p>The swerve drivetrain and the mechanisms share a bus, and CTRE requires every swerve device to
 * be on the same one. That bus used to be named in two places -- here and in the generated tuner
 * constants -- which is a silent failure waiting to happen: if the two drift apart the drivetrain
 * comes up split from its Pigeon rather than failing outright. Both now reference {@link #CANIVORE}.
 *
 * <p>Simulation collapses every bus onto Phoenix's single default simulated bus. Device IDs are
 * unique across all four real buses, so nothing collides when they share one in sim.
 */
public final class CanBuses {
  /**
   * Name the CANivore is registered under in Phoenix Tuner. If the drivetrain and mechanisms come
   * up with "CANbus Failed to Connect", this string is the first thing to check -- it has to match
   * what Tuner shows exactly.
   */
  public static final String CANIVORE_NAME = "CANivore";

  /** Swerve drivetrain (0-12, 51, Pigeon 0) and the shooter mechanisms (21-29). */
  public static final CANBus CANIVORE =
      Utils.isSimulation() ? new CANBus("") : new CANBus(CANIVORE_NAME);

  /** SystemCore can_s4: intake rollers, IDs 31 and 32. */
  public static final CANBus ROLLERS =
      Utils.isSimulation() ? new CANBus("") : CANBus.systemcore(4);

  /** SystemCore can_s1: the CANrange used as a beam break, ID 0. */
  public static final CANBus SENSORS =
      Utils.isSimulation() ? new CANBus("") : CANBus.systemcore(1);

  private CanBuses() {}
}
