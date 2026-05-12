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

import static edu.wpi.first.units.Units.*;

/**
 * Flywheel mechanism driven by a TalonFX.
 *
 * <p>Supports velocity control with configurable soft limits, SysId characterization,
 * and WPILib simulation via {@link FlywheelSim}.
 */
public class Flywheel extends SmartMechanism {

    private static final AngularVelocity DEFAULT_TOLERANCE = RPM.of(50);

    private final FlywheelConfig flyWheelConfig;
    private final FlywheelSim flywheelSim;
    private final SysIdRoutine sysIdRoutine;

    private AngularVelocity setpoint = RotationsPerSecond.of(0);
    private double simPositionRotations = 0.0;

    /** Constructs the flywheel and applies motor config. */
    public Flywheel(FlywheelConfig flyWheelConfig) {
        super(flyWheelConfig.motor, flyWheelConfig.resolveLogPrefix());
        flyWheelConfig.applyBuilt();
        this.flyWheelConfig = flyWheelConfig;

        if (RobotBase.isSimulation() && flyWheelConfig.motorModel != null
                && flyWheelConfig.diameter != null && flyWheelConfig.mass != null) {
            flywheelSim = new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            flyWheelConfig.motorModel,
                            flyWheelConfig.getMOI(),
                            motor.getRuntimeInfo().gearing()),
                    flyWheelConfig.motorModel);
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
     * Spins the flywheel at {@code velocity} continuously.
     * The command never finishes on its own; use {@link #runTo} to wait until up to speed.
     */
    public Command run(AngularVelocity velocity) {
        return Commands.run(() -> setVelocity(velocity), getSubsystem())
                .withName("FlyWheel.run(" + velocity.in(RPM) + " rpm)");
    }

    /** Spins the flywheel to {@code velocity} and finishes once {@link #atVelocity()} is true. */
    public Command runTo(AngularVelocity velocity) {
        return Commands.run(() -> setVelocity(velocity), getSubsystem())
                .until(this::atVelocity)
                .withName("FlyWheel.runTo(" + velocity.in(RPM) + " rpm)");
    }

    /** Sends a velocity setpoint, clamped by any configured soft limits. */
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

    /** Returns the current mechanism velocity from the motor's feedback sensor. */
    public AngularVelocity getVelocity() {
        return motor.getMechanismVelocity();
    }

    /**
     * Returns the tangential surface velocity based on the configured wheel diameter.
     * Returns zero if no diameter was configured.
     */
    public LinearVelocity getSurfaceVelocity() {
        if (flyWheelConfig.diameter == null)
            return MetersPerSecond.of(0.0);
        double radiusMeters = flyWheelConfig.diameter.in(Meters) / 2.0;
        return MetersPerSecond.of(getVelocity().in(RotationsPerSecond) * 2.0 * Math.PI * radiusMeters);
    }

    /** Returns {@code true} when the flywheel is within tolerance of the most recent setpoint. */
    public boolean atVelocity() {
        AngularVelocity tolerance = flyWheelConfig.velocityTolerance != null
                ? flyWheelConfig.velocityTolerance
                : DEFAULT_TOLERANCE;
        return atVelocity(setpoint, tolerance);
    }

    /** Returns {@code true} when the flywheel is within {@code tolerance} of {@code target}. */
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
        if (flywheelSim == null)
            return;
        if (Logger.hasReplaySource())
            return;

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
        Logger.recordOutput(logPrefix + "AtVelocity", atVelocity());

        Command active = getSubsystem().getCurrentCommand();
        Logger.recordOutput(logPrefix + "ActiveCommand", active != null ? active.getName() : "None");
    }
}
