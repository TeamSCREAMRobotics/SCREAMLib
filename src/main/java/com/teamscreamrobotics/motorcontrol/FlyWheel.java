package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;

public class FlyWheel extends SmartMechanism {

    private static final AngularVelocity DEFAULT_TOLERANCE = RPM.of(50);

    private final FlyWheelConfig flyWheelConfig;
    private final FlywheelSim flywheelSim;
    private final SysIdRoutine sysIdRoutine;

    private AngularVelocity setpoint = RotationsPerSecond.of(0);
    private double simPositionRotations = 0.0;

    public FlyWheel(FlyWheelConfig flyWheelConfig) {
        super(flyWheelConfig.motor, flyWheelConfig.resolveLogPrefix());
        this.flyWheelConfig = flyWheelConfig;

        if (RobotBase.isSimulation() && config.motorModel != null
                && flyWheelConfig.diameter != null && flyWheelConfig.mass != null) {
            flywheelSim = new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            config.motorModel,
                            flyWheelConfig.getMOI(),
                            config.gearing),
                    config.motorModel);
        } else {
            flywheelSim = null;
        }

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (voltage) -> motor.setVoltage(voltage),
                        (log) -> {
                            Logger.recordOutput(logPrefix + "SysId/Voltage",
                                    motor.getVoltage().in(Volts));
                            Logger.recordOutput(logPrefix + "SysId/Velocity",
                                    motor.getMechanismVelocity().in(RotationsPerSecond));
                            Logger.recordOutput(logPrefix + "SysId/Position",
                                    motor.getMechanismPosition().in(Rotations));
                        },
                        config.subsystem));
    }

    public Command run(AngularVelocity velocity) {
        return Commands.run(() -> setVelocity(velocity), config.subsystem)
                .withName("FlyWheel.run(" + velocity.in(RPM) + " rpm)");
    }

    public Command runTo(AngularVelocity velocity) {
        return Commands.run(() -> setVelocity(velocity), config.subsystem)
                .until(this::atVelocity)
                .withName("FlyWheel.runTo(" + velocity.in(RPM) + " rpm)");
    }

    public void setVelocity(AngularVelocity velocity) {
        double rps = velocity.in(RotationsPerSecond);
        if (flyWheelConfig.lowerSoftLimit != null) {
            rps = Math.max(rps, flyWheelConfig.lowerSoftLimit.in(RotationsPerSecond));
        }
        if (flyWheelConfig.upperSoftLimit != null) {
            rps = Math.min(rps, flyWheelConfig.upperSoftLimit.in(RotationsPerSecond));
        }
        this.setpoint = RotationsPerSecond.of(rps);
        motor.setVelocity(this.setpoint);
    }

    public AngularVelocity getVelocity() {
        return motor.getMechanismVelocity();
    }

    public LinearVelocity getSurfaceVelocity() {
        double radiusMeters = flyWheelConfig.diameter.in(Meters) / 2.0;
        return MetersPerSecond.of(getVelocity().in(RotationsPerSecond) * 2.0 * Math.PI * radiusMeters);
    }

    public boolean atVelocity() {
        return atVelocity(setpoint, DEFAULT_TOLERANCE);
    }

    public boolean atVelocity(AngularVelocity target, AngularVelocity tolerance) {
        return Math.abs(getVelocity().in(RPM) - target.in(RPM)) <= tolerance.in(RPM);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void simIterate() {
        if (flywheelSim == null) return;

        motor.simIterate(0.020);

        flywheelSim.setInputVoltage(motor.getVoltage().in(Volts));
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
        Logger.recordOutput(logPrefix + "AtVelocity", atVelocity());

        Command active = config.subsystem.getCurrentCommand();
        Logger.recordOutput(logPrefix + "ActiveCommand", active != null ? active.getName() : "None");
    }
}
