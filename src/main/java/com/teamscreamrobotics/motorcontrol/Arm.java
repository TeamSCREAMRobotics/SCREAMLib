package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.*;

public class Arm extends SmartMechanism {

    private static final Angle DEFAULT_TOLERANCE = Degrees.of(1.0);

    private final SingleJointedArmSim armSim;

    private Angle setpoint;

    public Arm(ArmConfig armConfig) {
        super(armConfig.motor, armConfig.resolveLogPrefix());
        this.setpoint = armConfig.startingPosition;

        motor.resetEncoder(armConfig.startingPosition);

        if (RobotBase.isSimulation() && armConfig.length != null && armConfig.mass != null
                && armConfig.hardLimitMin != null && armConfig.hardLimitMax != null
                && config.motorModel != null) {
            double lengthMeters = armConfig.length.in(Meters);
            double massKg = armConfig.mass.in(Kilograms);
            double j = (1.0 / 3.0) * massKg * lengthMeters * lengthMeters;
            armSim = new SingleJointedArmSim(
                    config.motorModel,
                    config.gearing,
                    j,
                    lengthMeters,
                    armConfig.hardLimitMin.in(Radians),
                    armConfig.hardLimitMax.in(Radians),
                    true,
                    armConfig.startingPosition.in(Radians));
        } else {
            armSim = null;
        }
    }

    public Command run(Angle angle) {
        return Commands.run(() -> setAngle(angle), config.subsystem)
                .withName("Arm.run(" + angle.in(Degrees) + " deg)");
    }

    public Command runTo(Angle angle) {
        return Commands.run(() -> setAngle(angle), config.subsystem)
                .until(this::atAngle)
                .withName("Arm.runTo(" + angle.in(Degrees) + " deg)");
    }

    public void setAngle(Angle angle) {
        this.setpoint = angle;
        motor.setPosition(angle);
    }

    public Angle getAngle() {
        return motor.getMechanismPosition();
    }

    public boolean atAngle() {
        return atAngle(setpoint, DEFAULT_TOLERANCE);
    }

    public boolean atAngle(Angle target, Angle tolerance) {
        return Math.abs(getAngle().in(Degrees) - target.in(Degrees)) <= tolerance.in(Degrees);
    }

    @Override
    public void simIterate() {
        if (armSim == null) return;

        motor.simIterate(0.020);

        armSim.setInputVoltage(motor.getVoltage().in(Volts));
        armSim.update(0.020);

        motor.simUpdate(
                Radians.of(armSim.getAngleRads()),
                RadiansPerSecond.of(armSim.getVelocityRadPerSec()));
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
