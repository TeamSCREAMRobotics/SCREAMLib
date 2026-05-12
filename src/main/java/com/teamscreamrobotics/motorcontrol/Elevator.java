package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

/**
 * Linear elevator mechanism driven by a TalonFX.
 *
 * <p>Supports position control with or without a MotionMagic profile, SysId characterization,
 * gravity characterization, and WPILib simulation via {@link ElevatorSim}.
 * Heights are measured as linear distance from the home position to the carriage.
 */
public class Elevator extends SmartMechanism {

    private static final Distance DEFAULT_TOLERANCE = Meters.of(0.01);

    private final ElevatorConfig elevatorConfig;
    private final ElevatorSim elevatorSim;
    private final SysIdRoutine sysIdRoutine;

    private Distance setpoint;

    /** Constructs the elevator, applies motor config, and seeds the encoder to the starting height. */
    public Elevator(ElevatorConfig elevatorConfig) {
        super(elevatorConfig.motor, elevatorConfig.resolveLogPrefix());
        elevatorConfig.applyBuilt();
        this.elevatorConfig = elevatorConfig;
        this.setpoint = elevatorConfig.startingHeight;

        double circumference = elevatorConfig.mechanismCircumference.in(Meters);
        motor.resetEncoder(Rotation2d.fromRotations(elevatorConfig.startingHeight.in(Meters) / circumference));

        if (RobotBase.isSimulation()
                && elevatorConfig.mass != null
                && elevatorConfig.hardLimitMin != null && elevatorConfig.hardLimitMax != null
                && elevatorConfig.motorModel != null) {
            double drumRadius = circumference / (2.0 * Math.PI);
            elevatorSim = new ElevatorSim(
                    elevatorConfig.motorModel,
                    motor.getRuntimeInfo().gearing(),
                    elevatorConfig.mass.in(Kilograms),
                    drumRadius,
                    elevatorConfig.hardLimitMin.in(Meters),
                    elevatorConfig.hardLimitMax.in(Meters),
                    true,
                    elevatorConfig.startingHeight.in(Meters));
        } else {
            elevatorSim = null;
        }

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (voltage) -> motor.setVoltage(voltage),
                        (log) -> {
                            Logger.recordOutput(logPrefix + "SysId/Voltage",
                                    motor.getVoltage().in(Volts));
                            Logger.recordOutput(logPrefix + "SysId/VelocityMPS",
                                    motor.getLinearVelocity().in(MetersPerSecond));
                            Logger.recordOutput(logPrefix + "SysId/HeightMeters",
                                    getHeight().in(Meters));
                        },
                        getSubsystem()));
    }

    /**
     * Runs the elevator to {@code height} continuously using MotionMagic profiled position control.
     * The command never finishes on its own; use {@link #runToWithProfile} for a one-shot move.
     */
    public Command runWithProfile(Distance height) {
        return Commands.run(() -> setHeightWithProfile(height), getSubsystem())
                .withName("Elevator.runWithProfile(" + height.in(Meters) + " m)");
    }

    /** Runs the elevator to {@code height} via MotionMagic and finishes once {@link #atHeight()} is true. */
    public Command runToWithProfile(Distance height) {
        return Commands.run(() -> setHeightWithProfile(height), getSubsystem())
                .until(this::atHeight)
                .withName("Elevator.runToWithProfile(" + height.in(Meters) + " m)");
    }

    /**
     * Runs the elevator to {@code height} continuously using direct position control (no profile).
     * The command never finishes on its own; use {@link #runTo} for a one-shot move.
     */
    public Command run(Distance height) {
        return Commands.run(() -> setHeight(height), getSubsystem())
                .withName("Elevator.run(" + height.in(Meters) + " m)");
    }

    /** Runs the elevator to {@code height} via direct position control and finishes once {@link #atHeight()} is true. */
    public Command runTo(Distance height) {
        return Commands.run(() -> setHeight(height), getSubsystem())
                .until(this::atHeight)
                .withName("Elevator.runTo(" + height.in(Meters) + " m)");
    }

    /** Sends a MotionMagic profiled linear position setpoint and updates the stored setpoint. */
    public void setHeightWithProfile(Distance height) {
        this.setpoint = height;
        motor.setLinearPositionProfiled(height);
    }

    /** Sends a direct linear position setpoint (no profile) and updates the stored setpoint. */
    public void setHeight(Distance height) {
        this.setpoint = height;
        motor.setLinearPosition(height);
    }

    /** Returns the current carriage height converted from motor rotations via mechanism circumference. */
    public Distance getHeight() {
        return motor.getLinearPosition();
    }

    /** Returns {@code true} when the elevator is within tolerance of the most recent setpoint. */
    public boolean atHeight() {
        Distance tolerance = elevatorConfig.positionTolerance != null
                ? elevatorConfig.positionTolerance : DEFAULT_TOLERANCE;
        return atHeight(setpoint, tolerance);
    }

    /** Returns {@code true} when the elevator is within {@code tolerance} of {@code target}. */
    public boolean atHeight(Distance target, Distance tolerance) {
        return Math.abs(getHeight().in(Meters) - target.in(Meters)) <= tolerance.in(Meters);
    }

    // ── Characterization ──────────────────────────────────────────────────────

    /**
     * Ramps open-loop voltage down while the elevator holds position to measure {@code kG}.
     * The measured holding voltage is logged to {@code GravityChar/KgEstimate}.
     */
    public Command gravityCharacterization() {
        return voltageRampDownCommand(1.5, 0.01, 0.05, 10, kg -> {
            Logger.recordOutput(logPrefix + "GravityChar/KgEstimate", kg);
            Logger.recordOutput(logPrefix + "GravityChar/Complete", true);
        }).withTimeout(10.0).withName("Elevator GravityCharacterization");
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
        if (elevatorSim == null) return;
        if (Logger.hasReplaySource()) return;

        elevatorSim.setInputVoltage(motor.getSimVoltage());
        elevatorSim.update(0.020);

        double circ = elevatorConfig.mechanismCircumference.in(Meters);
        motor.simUpdate(
                Rotation2d.fromRotations(elevatorSim.getPositionMeters() / circ),
                RotationsPerSecond.of(elevatorSim.getVelocityMetersPerSecond() / circ));
    }

    @Override
    public void updateTelemetry() {
        processInputs();

        Logger.recordOutput(logPrefix + "HeightMeters", getHeight().in(Meters));
        Logger.recordOutput(logPrefix + "SetpointMeters", setpoint.in(Meters));
        Logger.recordOutput(logPrefix + "AtHeight", atHeight());

        Command active = getSubsystem().getCurrentCommand();
        Logger.recordOutput(logPrefix + "ActiveCommand", active != null ? active.getName() : "None");
    }
}
