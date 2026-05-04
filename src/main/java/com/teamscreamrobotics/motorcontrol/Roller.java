package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public class Roller extends SmartMechanism {

    private static final double DEFAULT_MASS_KG = 0.1;

    private final RollerConfig rollerConfig;
    private final FlywheelSim flywheelSim;
    private double simPositionRotations = 0.0;

    public Roller(RollerConfig rollerConfig) {
        super(rollerConfig.motor, rollerConfig.resolveLogPrefix());
        this.rollerConfig = rollerConfig;

        if (RobotBase.isSimulation() && config.motorModel != null && rollerConfig.diameter != null) {
            double radius = rollerConfig.diameter.in(Meters) / 2.0;
            double moi = 0.5 * DEFAULT_MASS_KG * radius * radius;
            flywheelSim = new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(config.motorModel, moi, config.gearing),
                    config.motorModel);
        } else {
            flywheelSim = null;
        }
    }

    public Command run(AngularVelocity velocity) {
        return Commands.run(() -> setVelocity(velocity), config.subsystem)
                .withName("Roller.run(" + velocity.in(RPM) + " rpm)");
    }

    public Command run(Voltage voltage) {
        return Commands.run(() -> setVoltage(voltage), config.subsystem)
                .withName("Roller.run(" + voltage.in(Volts) + " V)");
    }

    public Command stop() {
        return Commands.runOnce(motor::stop, config.subsystem)
                .withName("Roller.stop()");
    }

    public void setVelocity(AngularVelocity velocity) {
        double rps = velocity.in(RotationsPerSecond);
        if (rollerConfig.lowerSoftLimit != null) {
            rps = Math.max(rps, rollerConfig.lowerSoftLimit.in(RotationsPerSecond));
        }
        if (rollerConfig.upperSoftLimit != null) {
            rps = Math.min(rps, rollerConfig.upperSoftLimit.in(RotationsPerSecond));
        }
        motor.setVelocity(RotationsPerSecond.of(rps));
    }

    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    public AngularVelocity getVelocity() {
        return motor.getMechanismVelocity();
    }

    public LinearVelocity getSurfaceVelocity() {
        if (rollerConfig.diameter == null) return MetersPerSecond.of(0.0);
        double radiusMeters = rollerConfig.diameter.in(Meters) / 2.0;
        return MetersPerSecond.of(getVelocity().in(RotationsPerSecond) * 2.0 * Math.PI * radiusMeters);
    }

    public boolean atVelocity(AngularVelocity target, AngularVelocity tolerance) {
        return Math.abs(getVelocity().in(RPM) - target.in(RPM)) <= tolerance.in(RPM);
    }

    @Override
    public void simIterate() {
        if (flywheelSim == null) return;
        if (Logger.hasReplaySource()) return;

        flywheelSim.setInputVoltage(motor.getSimVoltage());
        flywheelSim.update(0.020);

        double velocityRps = flywheelSim.getAngularVelocityRPM() / 60.0;
        simPositionRotations += velocityRps * 0.020;

        motor.simUpdate(
                Rotations.of(simPositionRotations),
                RotationsPerSecond.of(velocityRps));
    }

    @Override
    public void updateTelemetry() {
        processInputs();

        Logger.recordOutput(logPrefix + "VelocityRPM", getVelocity().in(RPM));
        Logger.recordOutput(logPrefix + "SurfaceVelocityMPS", getSurfaceVelocity().in(MetersPerSecond));

        Command active = config.subsystem.getCurrentCommand();
        Logger.recordOutput(logPrefix + "ActiveCommand", active != null ? active.getName() : "None");
    }
}
