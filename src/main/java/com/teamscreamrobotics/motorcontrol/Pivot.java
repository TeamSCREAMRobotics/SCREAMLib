package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.*;

public class Pivot extends SmartMechanism {

    private static final Angle DEFAULT_TOLERANCE = Degrees.of(1.0);

    private final DCMotorSim pivotSim;

    private Angle setpoint;

    public Pivot(PivotConfig pivotConfig) {
        super(pivotConfig.motor, pivotConfig.resolveLogPrefix());
        this.setpoint = pivotConfig.startingPosition;

        motor.resetEncoder(pivotConfig.startingPosition);

        if (RobotBase.isSimulation() && config.motorModel != null) {
            pivotSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            config.motorModel,
                            pivotConfig.moiKgMetersSquared,
                            config.gearing),
                    config.motorModel);
        } else {
            pivotSim = null;
        }
    }

    public Command run(Angle angle) {
        return Commands.run(() -> setAngle(angle), config.subsystem)
                .withName("Pivot.run(" + angle.in(Degrees) + " deg)");
    }

    public Command runTo(Angle angle) {
        return Commands.run(() -> setAngle(angle), config.subsystem)
                .until(this::atAngle)
                .withName("Pivot.runTo(" + angle.in(Degrees) + " deg)");
    }

    public void setAngle(Angle angle) {
        this.setpoint = angle;
        motor.setPosition(angle);
    }

    public Angle getAngle() {
        return motor.getMechanismPosition();
    }

    public boolean atAngle() {
        Angle tolerance = config.positionTolerance != null ? config.positionTolerance : DEFAULT_TOLERANCE;
        return atAngle(setpoint, tolerance);
    }

    public boolean atAngle(Angle target, Angle tolerance) {
        return Math.abs(getAngle().in(Degrees) - target.in(Degrees)) <= tolerance.in(Degrees);
    }

    @Override
    public void simIterate() {
        if (pivotSim == null) return;
        if (Logger.hasReplaySource()) return;

        pivotSim.setInputVoltage(motor.getSimVoltage());
        pivotSim.update(0.020);

        motor.simUpdate(
                Rotations.of(pivotSim.getAngularPositionRotations()),
                RotationsPerSecond.of(pivotSim.getAngularVelocityRPM() / 60.0));
    }

    @Override
    public void updateTelemetry() {
        processInputs();

        Logger.recordOutput(logPrefix + "AngleDegrees", getAngle().in(Degrees));
        Logger.recordOutput(logPrefix + "SetpointDegrees", setpoint.in(Degrees));
        Logger.recordOutput(logPrefix + "AtAngle", atAngle());

        Command active = config.subsystem.getCurrentCommand();
        Logger.recordOutput(logPrefix + "ActiveCommand", active != null ? active.getName() : "None");
    }
}
