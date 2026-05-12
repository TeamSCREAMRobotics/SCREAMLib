package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

/**
 * Roller mechanism driven by a TalonFX.
 *
 * <p>Supports both velocity and open-loop voltage control, configurable velocity soft limits,
 * SysId characterization, and WPILib simulation via a flywheel model.
 */
public class Roller extends SmartMechanism {

    private static final double DEFAULT_MASS_KG = 0.1;

    private final RollerConfig rollerConfig;
    private final FlywheelSim flywheelSim;
    private final SysIdRoutine sysIdRoutine;
    private double simPositionRotations = 0.0;

    /** Constructs the roller and applies motor config. */
    public Roller(RollerConfig rollerConfig) {
        super(rollerConfig.motor, rollerConfig.resolveLogPrefix());
        rollerConfig.applyBuilt();
        this.rollerConfig = rollerConfig;

        if (RobotBase.isSimulation() && rollerConfig.motorModel != null && rollerConfig.diameter != null) {
            double radius = rollerConfig.diameter.in(Meters) / 2.0;
            double massKg = rollerConfig.mass != null ? rollerConfig.mass.in(Kilograms) : DEFAULT_MASS_KG;
            double moi = 0.5 * massKg * radius * radius;
            flywheelSim = new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(rollerConfig.motorModel, moi, motor.getRuntimeInfo().gearing()),
                    rollerConfig.motorModel);
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
                                    motor.getMechanismPosition().getRotations());
                        },
                        getSubsystem()));
    }

    /**
     * Runs the roller at the given velocity continuously, clamped by any configured soft limits.
     */
    public Command run(AngularVelocity velocity) {
        return Commands.run(() -> setVelocity(velocity), getSubsystem())
                .withName("Roller.run(" + velocity.in(RPM) + " rpm)");
    }

    /** Runs the roller at a fixed open-loop voltage continuously. */
    public Command run(Voltage voltage) {
        return Commands.run(() -> setVoltage(voltage), getSubsystem())
                .withName("Roller.run(" + voltage.in(Volts) + " V)");
    }

    /** Stops the roller and ends immediately. */
    public Command stop() {
        return Commands.runOnce(motor::stop, getSubsystem())
                .withName("Roller.stop()");
    }

    /** Sends a velocity setpoint, clamped by any configured soft limits. */
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

    /** Applies an open-loop voltage directly to the motor. */
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    /** Returns the current mechanism velocity from the motor's feedback sensor. */
    public AngularVelocity getVelocity() {
        return motor.getMechanismVelocity();
    }

    /**
     * Returns the tangential surface velocity based on the configured wheel diameter.
     * Returns zero if no diameter was configured.
     */
    public LinearVelocity getSurfaceVelocity() {
        if (rollerConfig.diameter == null) return MetersPerSecond.of(0.0);
        double radiusMeters = rollerConfig.diameter.in(Meters) / 2.0;
        return MetersPerSecond.of(getVelocity().in(RotationsPerSecond) * 2.0 * Math.PI * radiusMeters);
    }

    /** Returns {@code true} when the roller is within {@code tolerance} of {@code target}. */
    public boolean atVelocity(AngularVelocity target, AngularVelocity tolerance) {
        return Math.abs(getVelocity().in(RPM) - target.in(RPM)) <= tolerance.in(RPM);
    }

    /** Returns a quasistatic SysId characterization command in the given direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /** Returns a dynamic (step voltage) SysId characterization command in the given direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
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
                Rotation2d.fromRotations(simPositionRotations),
                RotationsPerSecond.of(velocityRps));
    }

    @Override
    public void updateTelemetry() {
        processInputs();

        Logger.recordOutput(logPrefix + "VelocityRPM", getVelocity().in(RPM));
        Logger.recordOutput(logPrefix + "SurfaceVelocityMPS", getSurfaceVelocity().in(MetersPerSecond));

        Command active = getSubsystem().getCurrentCommand();
        Logger.recordOutput(logPrefix + "ActiveCommand", active != null ? active.getName() : "None");
    }
}
