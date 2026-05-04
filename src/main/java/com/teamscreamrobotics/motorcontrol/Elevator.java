package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SmartMechanism {

    private static final Distance DEFAULT_TOLERANCE = Meters.of(0.01);

    private final ElevatorSim elevatorSim;

    private Distance setpoint;

    public Elevator(ElevatorConfig elevatorConfig) {
        super(elevatorConfig.motor, elevatorConfig.resolveLogPrefix());
        this.setpoint = elevatorConfig.startingHeight;

        double circumference = config.mechanismCircumference.in(Meters);
        motor.resetEncoder(Rotations.of(elevatorConfig.startingHeight.in(Meters) / circumference));

        if (RobotBase.isSimulation() && elevatorConfig.mass != null
                && elevatorConfig.hardLimitMin != null && elevatorConfig.hardLimitMax != null
                && config.motorModel != null && config.mechanismCircumference != null) {
            double drumRadius = circumference / (2.0 * Math.PI);
            elevatorSim = new ElevatorSim(
                    config.motorModel,
                    config.gearing,
                    elevatorConfig.mass.in(Kilograms),
                    drumRadius,
                    elevatorConfig.hardLimitMin.in(Meters),
                    elevatorConfig.hardLimitMax.in(Meters),
                    true,
                    elevatorConfig.startingHeight.in(Meters));
        } else {
            elevatorSim = null;
        }
    }

    public Command run(Distance height) {
        return Commands.run(() -> setHeight(height), config.subsystem)
                .withName("Elevator.run(" + height.in(Meters) + " m)");
    }

    public Command runTo(Distance height) {
        return Commands.run(() -> setHeight(height), config.subsystem)
                .until(this::atHeight)
                .withName("Elevator.runTo(" + height.in(Meters) + " m)");
    }

    public void setHeight(Distance height) {
        this.setpoint = height;
        motor.setLinearPosition(height);
    }

    public Distance getHeight() {
        return motor.getLinearPosition();
    }

    public boolean atHeight() {
        return atHeight(setpoint, DEFAULT_TOLERANCE);
    }

    public boolean atHeight(Distance target, Distance tolerance) {
        return Math.abs(getHeight().in(Meters) - target.in(Meters)) <= tolerance.in(Meters);
    }

    @Override
    public void simIterate() {
        if (elevatorSim == null) return;

        motor.simIterate(0.020);

        elevatorSim.setInputVoltage(motor.getVoltage().in(Volts));
        elevatorSim.update(0.020);

        double circumference = config.mechanismCircumference.in(Meters);
        motor.simUpdate(
                Rotations.of(elevatorSim.getPositionMeters() / circumference),
                RotationsPerSecond.of(elevatorSim.getVelocityMetersPerSecond() / circumference));
    }

    @Override
    public void updateTelemetry() {
        processInputs();

        Logger.recordOutput(logPrefix + "HeightMeters", getHeight().in(Meters));
        Logger.recordOutput(logPrefix + "SetpointMeters", setpoint.in(Meters));
        Logger.recordOutput(logPrefix + "AtHeight", atHeight());

        Command active = config.subsystem.getCurrentCommand();
        Logger.recordOutput(logPrefix + "ActiveCommand", active != null ? active.getName() : "None");
    }
}
