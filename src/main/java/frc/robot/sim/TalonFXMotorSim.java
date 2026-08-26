package frc.robot.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

/** Small physics adapter that feeds a TalonFX's simulated rotor sensors. */
final class TalonFXMotorSim {
  private final TalonFX motor;
  private final DCMotorSim physics;
  private final double inversionMultiplier;

  TalonFXMotorSim(
      TalonFX motor, double rotorInertiaKgMetersSquared, boolean clockwisePositive) {
    this.motor = motor;
    inversionMultiplier = clockwisePositive ? -1.0 : 1.0;
    DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
    physics = new DCMotorSim(
        Models.singleJointedArmFromPhysicalConstants(
            gearbox, rotorInertiaKgMetersSquared, 1.0),
        gearbox);
  }

  double update(double periodSeconds, double supplyVoltage) {
    var state = motor.getSimState();
    state.setSupplyVoltage(supplyVoltage);
    // Phoenix reports physical motor voltage after inversion, while closed-loop requests and
    // sensor values use the configured positive direction. Convert in and back out consistently.
    physics.setInputVoltage(state.getMotorVoltage() * inversionMultiplier);
    physics.update(periodSeconds);
    state.setRawRotorPosition(
        physics.getAngularPosition() / (2.0 * Math.PI) * inversionMultiplier);
    state.setRotorVelocity(
        physics.getAngularVelocity() / (2.0 * Math.PI) * inversionMultiplier);
    return Math.abs(physics.getCurrentDraw());
  }
}
