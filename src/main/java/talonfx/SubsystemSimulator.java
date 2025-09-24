package talonfx;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import talonfx.SubsystemConfiguration.ArmConfig;
import talonfx.SubsystemConfiguration.BaseConfig;
import talonfx.SubsystemConfiguration.ElevatorConfig;
import talonfx.SubsystemConfiguration.FlywheelConfig;
import talonfx.SubsystemConfiguration.GenericConfig;
import util.SimUtil;

public class SubsystemSimulator {
  Notifier simThread = null;
  double lastSimTime;
  double deltaTime;

  BaseConfig config;

  Consumer<Double> updateConsumer;
  Consumer<Double> voltageConsumer;
  DoubleSupplier positionSupplier;
  DoubleSupplier velocitySupplier;

  double previousVelocity = 0;

  double acceleration;

  private SubsystemSimulator(DCMotorSim sim, GenericConfig config) {
    this.config = config;
    updateConsumer = sim::update;
    voltageConsumer = (value) -> sim.setInput(0, value);
    positionSupplier = () -> sim.getAngularPositionRotations() * sim.getGearing();
    velocitySupplier = () -> (sim.getAngularVelocityRPM() / 60.0) * sim.getGearing();
  }

  private SubsystemSimulator(ElevatorSim sim, ElevatorConfig config) {
    this.config = config;
    updateConsumer = sim::update;
    voltageConsumer = (value) -> sim.setInput(0, value);
    positionSupplier =
        () ->
            (sim.getPositionMeters() / config.spoolCircumference.getMeters())
                * config.rotorToSensorRatio
                * config.sensorToMechRatio;
    velocitySupplier =
        () ->
            (sim.getVelocityMetersPerSecond() / config.spoolCircumference.getMeters())
                * config.rotorToSensorRatio
                * config.sensorToMechRatio;
  }

  private SubsystemSimulator(SingleJointedArmSim sim, ArmConfig config) {
    this.config = config;
    updateConsumer = sim::update;
    voltageConsumer = (value) -> sim.setInput(0, value);
    positionSupplier =
        () ->
            Units.radiansToRotations(sim.getAngleRads())
                * config.rotorToSensorRatio
                * config.sensorToMechRatio;
    velocitySupplier =
        () ->
            Units.radiansToRotations(sim.getVelocityRadPerSec())
                * config.rotorToSensorRatio
                * config.sensorToMechRatio;
  }

  private SubsystemSimulator(FlywheelSim sim, FlywheelConfig config) {
    this.config = config;
    updateConsumer = sim::update;
    voltageConsumer = (value) -> sim.setInput(0, value);
    positionSupplier = () -> 0.0;
    velocitySupplier = () -> (sim.getAngularVelocityRPM() / 60.0) * sim.getGearing();
  }

  public void update(double deltaTime) {
    updateConsumer.accept(deltaTime);
  }

  public void setInputVoltage(double inputVoltage) {
    voltageConsumer.accept(inputVoltage);
  }

  public double getPosition() {
    return positionSupplier.getAsDouble();
  }

  public double getVelocity() {
    return velocitySupplier.getAsDouble();
  }

  public double getAcceleration() {
    return acceleration;
  }

  public void startSimThread(String name) {
    simThread =
        new Notifier(
            () -> {
              final double currentTime = Timer.getFPGATimestamp();
              deltaTime = currentTime - lastSimTime;

              update(deltaTime);

              acceleration = (getVelocity() - previousVelocity) / deltaTime;
              previousVelocity = getVelocity();

              lastSimTime = currentTime;
            });
    simThread.setName(config.name + " Simulator Thread");
    simThread.startPeriodic(config.simulationPeriodSec);
  }

  public static class GenericSimulator extends SubsystemSimulator {
    GenericSimulator(GenericConfig config) {
      super(
          SimUtil.createDCMotorSim(
              DCMotor.getKrakenX60(1 + config.slaveConstants.length),
              config.rotorToSensorRatio * config.sensorToMechRatio,
              config.simConfig.JKgMetersSquaredMOI),
          config);
    }
  }

  public static class ArmSimulator extends SubsystemSimulator {
    ArmSimulator(ArmConfig config) {
      super(
          new SingleJointedArmSim(
              config.simConfig.motorType.get(1 + config.slaveConstants.length),
              config.rotorToSensorRatio * config.sensorToMechRatio,
              config.simConfig.JKgMetersSquaredMOI,
              config.simConfig.armLength.getMeters(),
              config.simConfig.minAngle.getRadians(),
              config.simConfig.maxAngle.getRadians(),
              config.simConfig.simulateGravity,
              config.simConfig.startingAngle.getRadians()),
          config);
    }
  }

  public static class ElevatorSimulator extends SubsystemSimulator {
    ElevatorSimulator(ElevatorConfig config) {
      super(
          new ElevatorSim(
              config.simConfig.motorType.get(1 + config.slaveConstants.length),
              config.rotorToSensorRatio * config.sensorToMechRatio,
              config.simConfig.carriageMass.getKilograms(),
              config.spoolCircumference.getMeters(),
              config.simConfig.minHeight.getMeters(),
              config.simConfig.maxHeight.getMeters(),
              config.simConfig.simulateGravity,
              config.simConfig.startingHeight.getMeters()),
          config);
    }
  }

  public static class FlywheelSimulator extends SubsystemSimulator {
    FlywheelSimulator(FlywheelConfig config) {
      super(
          SimUtil.createFlywheelSim(
              config.simConfig.motorType.get(1 + config.slaveConstants.length),
              config.rotorToSensorRatio * config.sensorToMechRatio,
              config.simConfig.JKgMetersSquaredMOI),
          config);
    }
  }
}
