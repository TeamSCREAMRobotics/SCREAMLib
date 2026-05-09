package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SmartMechanism {

    private static final Distance DEFAULT_TOLERANCE = Meters.of(0.01);

    private final ElevatorSim elevatorSim;
    private final SysIdRoutine sysIdRoutine;

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
                        config.subsystem));
    }

    public Command runWithProfile(Distance height) {
        return Commands.run(() -> setHeightWithProfile(height), config.subsystem)
                .withName("Elevator.runWithProfile(" + height.in(Meters) + " m)");
    }

    public Command runToWithProfile(Distance height) {
        return Commands.run(() -> setHeightWithProfile(height), config.subsystem)
                .until(this::atHeight)
                .withName("Elevator.runToWithProfile(" + height.in(Meters) + " m)");
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

    public void setHeightWithProfile(Distance height) {
        this.setpoint = height;
        motor.setLinearPositionProfiled(height);
    }

    public void setHeight(Distance height) {
        this.setpoint = height;
        motor.setLinearPosition(height);
    }

    public Distance getHeight() {
        return motor.getLinearPosition();
    }

    public boolean atHeight() {
        if (config.positionTolerance != null) {
            double toleranceMeters = config.positionTolerance.in(Rotations)
                    * config.mechanismCircumference.in(Meters);
            return atHeight(setpoint, Meters.of(toleranceMeters));
        }
        return atHeight(setpoint, DEFAULT_TOLERANCE);
    }

    public boolean atHeight(Distance target, Distance tolerance) {
        return Math.abs(getHeight().in(Meters) - target.in(Meters)) <= tolerance.in(Meters);
    }

    // ── Characterization ──────────────────────────────────────────────────────

    /**
     * Ramps open-loop voltage upward from the elevator's current position until it holds.
     * The measured holding voltage is {@code kG}.
     *
     * <p>Read {@code KgEstimate} from AKit logs and plug it into your
     * {@code ElevatorFeedforward(kS, kG, kV, kA)} constructor.
     * Tune {@code kS} and {@code kV} via tuning mode or empirically.
     *
     * <p>WARNING: This command drives the mechanism with open-loop voltage.
     * Ensure soft limits are enabled and the mechanism is clear of
     * obstructions before running. The command will not stop automatically
     * if the mechanism hits a hard stop -- use with caution.
     * Recommended: run only in a controlled environment, not during competition.
     */
    public Command gravityCharacterization() {
        return voltageRampDownCommand(1.5, 0.01, 0.05, 10, kg -> {
            Logger.recordOutput(logPrefix + "GravityChar/KgEstimate", kg);
            Logger.recordOutput(logPrefix + "GravityChar/Complete", true);
        }).withTimeout(10.0)
        .withName("Elevator GravityCharacterization");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void simIterate() {
        if (elevatorSim == null) return;
        if (Logger.hasReplaySource()) return;

        elevatorSim.setInputVoltage(motor.getSimVoltage());
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
