package com.team4522.lib.sim;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimWrapper implements SimInterface{

    DCMotorSim dcSim;
    ElevatorSim elevSim;
    SingleJointedArmSim armSim;
    FlywheelSim flywheelSim;

    Consumer<Double> updateConsumer;
    Consumer<Double> voltageConsumer;
    DoubleSupplier positionSupplier;
    DoubleSupplier velocitySupplier;

    public SimWrapper(DCMotorSim sim){
        updateConsumer = sim::update;
        voltageConsumer = sim::setInputVoltage;
        positionSupplier = sim::getAngularPositionRotations;
        velocitySupplier = sim::getAngularVelocityRPM;
    }

    public SimWrapper(ElevatorSim sim){
        updateConsumer = sim::update;
        voltageConsumer = sim::setInputVoltage;
        positionSupplier = sim::getPositionMeters;
        velocitySupplier = sim::getVelocityMetersPerSecond;
    }

    public SimWrapper(SingleJointedArmSim sim){
        updateConsumer = sim::update;
        voltageConsumer = sim::setInputVoltage;
        positionSupplier = () -> Units.radiansToRotations(sim.getAngleRads());
        velocitySupplier = () -> Units.radiansToRotations(sim.getVelocityRadPerSec());
    }

    public SimWrapper(FlywheelSim sim){
        updateConsumer = sim::update;
        voltageConsumer = sim::setInputVoltage;
        positionSupplier = () -> 0.0;
        velocitySupplier = () -> (sim.getAngularVelocityRPM() / 60.0);
    }

    @Override
    public void update(double deltaTime){
        updateConsumer.accept(deltaTime);
    }

    @Override
    public void setInputVoltage(double inputVoltage){
        voltageConsumer.accept(inputVoltage);
    }

    @Override
    public double getPosition(){
        return positionSupplier.getAsDouble();
    }

    @Override
    public double getVelocity(){
        return velocitySupplier.getAsDouble();
    }
}
