package com.teamscreamrobotics.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.teamscreamrobotics.data.Length;

public class SimWrapper implements SimInterface {

  DCMotorSim dcSim;
  ElevatorSim elevSim;
  SingleJointedArmSim armSim;
  FlywheelSim flywheelSim;

  Consumer<Double> updateConsumer;
  Consumer<Double> voltageConsumer;
  DoubleSupplier positionSupplier;
  DoubleSupplier velocitySupplier;

  public SimWrapper(DCMotorSim sim) {
    updateConsumer = sim::update;
    voltageConsumer = (value) -> sim.setInput(0, value);
    positionSupplier = () -> sim.getAngularPositionRotations() * sim.getGearing();
    velocitySupplier = () -> (sim.getAngularVelocityRPM() / 60.0) * sim.getGearing();
  }

  public SimWrapper(ElevatorSim sim, Length spoolCircumference, double gearing) {
    updateConsumer = sim::update;
    voltageConsumer = (value) -> sim.setInput(0, value);
    positionSupplier = () -> (sim.getPositionMeters() / spoolCircumference.getMeters()) * gearing;
    velocitySupplier = () -> (sim.getVelocityMetersPerSecond() / spoolCircumference.getMeters()) * gearing;
  }

  public SimWrapper(SingleJointedArmSim sim, double gearing) {
    updateConsumer = sim::update;
    voltageConsumer = (value) -> sim.setInput(0, value);
    positionSupplier = () -> Units.radiansToRotations(sim.getAngleRads()) * gearing;
    velocitySupplier = () -> Units.radiansToRotations(sim.getVelocityRadPerSec()) * gearing;
  }

  public SimWrapper(FlywheelSim sim) {
    updateConsumer = sim::update;
    voltageConsumer = (value) -> sim.setInput(0, value);
    positionSupplier = () -> 0.0;
    velocitySupplier = () -> (sim.getAngularVelocityRPM() / 60.0) * sim.getGearing();
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
