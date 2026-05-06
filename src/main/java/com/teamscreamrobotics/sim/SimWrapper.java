package com.teamscreamrobotics.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.teamscreamrobotics.data.Length;

/**
 * Adapts WPILib simulation models ({@link DCMotorSim}, {@link ElevatorSim}, etc.) to the
 * {@link SimInterface} contract, normalizing position and velocity to rotations / rotations-per-second.
 */
@Deprecated(since = "2027")
public class SimWrapper implements SimInterface {

  DCMotorSim dcSim;
  ElevatorSim elevSim;
  SingleJointedArmSim armSim;
  FlywheelSim flywheelSim;

  Consumer<Double> updateConsumer;
  Consumer<Double> voltageConsumer;
  DoubleSupplier positionSupplier;
  DoubleSupplier velocitySupplier;

  /**
   * Wraps a {@link DCMotorSim}. Position and velocity are scaled by the model's internal gearing
   * to produce rotor rotations/RPS. Use the explicit-gearing overload if {@code sim} was built
   * with {@code gearing = 1.0} and gearing is tracked separately.
   *
   * @param sim the DC motor simulation model
   */
  public SimWrapper(DCMotorSim sim) {
    updateConsumer = sim::update;
    voltageConsumer = (value) -> sim.setInput(0, value);
    positionSupplier = () -> sim.getAngularPositionRotations() * sim.getGearing();
    velocitySupplier = () -> (sim.getAngularVelocityRPM() / 60.0) * sim.getGearing();
  }

  /**
   * Wraps a {@link DCMotorSim} with an explicit gear ratio. Use this when the sim model was
   * constructed with {@code gearing = 1.0} and the ratio is tracked in {@code TalonFXSubsystemSimConstants}.
   *
   * @param sim     the DC motor simulation model
   * @param gearing gear ratio from mechanism to motor (motor rotations per mechanism rotation)
   */
  public SimWrapper(DCMotorSim sim, double gearing) {
    updateConsumer = sim::update;
    voltageConsumer = (value) -> sim.setInput(0, value);
    positionSupplier = () -> sim.getAngularPositionRotations() * gearing;
    velocitySupplier = () -> (sim.getAngularVelocityRPM() / 60.0) * gearing;
  }

  /**
   * Wraps an {@link ElevatorSim}. Position and velocity are converted from meters to rotations/RPS
   * using the spool circumference and gear ratio.
   *
   * @param sim               the elevator simulation model
   * @param spoolCircumference circumference of the elevator spool
   * @param gearing           gear ratio between motor and spool
   */
  public SimWrapper(ElevatorSim sim, Length spoolCircumference, double gearing) {
    updateConsumer = sim::update;
    voltageConsumer = (value) -> sim.setInput(0, value);
    positionSupplier = () -> (sim.getPositionMeters() / spoolCircumference.getMeters()) * gearing;
    velocitySupplier = () -> (sim.getVelocityMetersPerSecond() / spoolCircumference.getMeters()) * gearing;
  }

  /**
   * Wraps a {@link SingleJointedArmSim}. Position and velocity are converted from radians to
   * rotations/RPS using the gear ratio.
   *
   * @param sim     the arm simulation model
   * @param gearing gear ratio between motor and arm joint
   */
  public SimWrapper(SingleJointedArmSim sim, double gearing) {
    updateConsumer = sim::update;
    voltageConsumer = (value) -> sim.setInput(0, value);
    positionSupplier = () -> Units.radiansToRotations(sim.getAngleRads()) * gearing;
    velocitySupplier = () -> Units.radiansToRotations(sim.getVelocityRadPerSec()) * gearing;
  }

  /**
   * Wraps a {@link FlywheelSim}. Position always returns {@code 0.0}; velocity is in RPS scaled
   * by the model's internal gearing. Use the explicit-gearing overload if the model gearing is 1.0.
   *
   * @param sim the flywheel simulation model
   */
  public SimWrapper(FlywheelSim sim) {
    updateConsumer = sim::update;
    voltageConsumer = (value) -> sim.setInput(0, value);
    positionSupplier = () -> 0.0;
    velocitySupplier = () -> (sim.getAngularVelocityRPM() / 60.0) * sim.getGearing();
  }

  /**
   * Wraps a {@link FlywheelSim} with an explicit gear ratio. Use this when the sim model was
   * constructed with {@code gearing = 1.0} and the ratio is tracked in {@code TalonFXSubsystemSimConstants}.
   *
   * @param sim     the flywheel simulation model
   * @param gearing gear ratio from mechanism to motor
   */
  public SimWrapper(FlywheelSim sim, double gearing) {
    updateConsumer = sim::update;
    voltageConsumer = (value) -> sim.setInput(0, value);
    positionSupplier = () -> 0.0;
    velocitySupplier = () -> (sim.getAngularVelocityRPM() / 60.0) * gearing;
  }

  @Override
  public void update(double deltaTime) {
    updateConsumer.accept(deltaTime);
  }

  @Override
  public void setInputVoltage(double inputVoltage) {
    voltageConsumer.accept(inputVoltage);
  }

  @Override
  public double getPosition() {
    return positionSupplier.getAsDouble();
  }

  @Override
  public double getVelocity() {
    return velocitySupplier.getAsDouble();
  }
}
