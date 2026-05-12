package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

/**
 * Single-joint arm mechanism driven by a TalonFX.
 *
 * <p>Supports position control with or without a MotionMagic profile, gravity
 * characterization, and WPILib simulation via {@link SingleJointedArmSim}.
 */
public class Arm extends SmartMechanism {

    private static final Rotation2d DEFAULT_TOLERANCE = Rotation2d.fromDegrees(1.0);

    private final ArmConfig armConfig;
    private final SingleJointedArmSim armSim;

    private Rotation2d setpoint;

    /** Constructs the arm, applies motor config, and seeds the encoder to the starting position. */
    public Arm(ArmConfig armConfig) {
        super(armConfig.motor, armConfig.resolveLogPrefix());
        armConfig.applyBuilt();
        this.armConfig = armConfig;
        this.setpoint = armConfig.startingPosition;

        motor.setHorizontalZeroRad(armConfig.horizontalZero.getRadians());
        motor.resetEncoder(armConfig.startingPosition);

        if (RobotBase.isSimulation()
                && armConfig.length != null && armConfig.mass != null
                && armConfig.hardLimitMin != null && armConfig.hardLimitMax != null
                && armConfig.motorModel != null) {
            double lengthMeters = armConfig.length.in(Meters);
            double massKg = armConfig.mass.in(Kilograms);
            double j = (1.0 / 3.0) * massKg * lengthMeters * lengthMeters;
            armSim = new SingleJointedArmSim(
                    armConfig.motorModel,
                    motor.getRuntimeInfo().gearing(),
                    j,
                    lengthMeters,
                    armConfig.hardLimitMin.getRadians(),
                    armConfig.hardLimitMax.getRadians(),
                    true,
                    armConfig.startingPosition.getRadians());
        } else {
            armSim = null;
        }
    }

    /**
     * Runs the arm to {@code angle} continuously using MotionMagic profiled position control.
     * The command never finishes on its own; use {@link #runToWithProfile} for a one-shot move.
     */
    public Command runWithProfile(Rotation2d angle) {
        return Commands.run(() -> setAngleWithProfile(angle), getSubsystem())
                .withName("Arm.runWithProfile(" + angle.getDegrees() + " deg)");
    }

    /** Runs the arm to {@code angle} via MotionMagic and finishes once {@link #atAngle()} is true. */
    public Command runToWithProfile(Rotation2d angle) {
        return Commands.run(() -> setAngleWithProfile(angle), getSubsystem())
                .until(this::atAngle)
                .withName("Arm.runToWithProfile(" + angle.getDegrees() + " deg)");
    }

    /**
     * Runs the arm to {@code angle} continuously using direct position control (no profile).
     * The command never finishes on its own; use {@link #runTo} for a one-shot move.
     */
    public Command run(Rotation2d angle) {
        return Commands.run(() -> setAngle(angle), getSubsystem())
                .withName("Arm.run(" + angle.getDegrees() + " deg)");
    }

    /** Runs the arm to {@code angle} via direct position control and finishes once {@link #atAngle()} is true. */
    public Command runTo(Rotation2d angle) {
        return Commands.run(() -> setAngle(angle), getSubsystem())
                .until(this::atAngle)
                .withName("Arm.runTo(" + angle.getDegrees() + " deg)");
    }

    /** Sends a MotionMagic profiled position setpoint and updates the stored setpoint. */
    public void setAngleWithProfile(Rotation2d angle) {
        this.setpoint = angle;
        motor.setPositionProfiled(angle);
    }

    /** Sends a direct position setpoint (no profile) and updates the stored setpoint. */
    public void setAngle(Rotation2d angle) {
        this.setpoint = angle;
        motor.setPosition(angle);
    }

    /** Returns the current mechanism angle from the motor's feedback sensor. */
    public Rotation2d getAngle() {
        return motor.getMechanismPosition();
    }

    /** Returns {@code true} when the arm is within tolerance of the most recent setpoint. */
    public boolean atAngle() {
        Rotation2d tolerance = armConfig.positionTolerance != null
                ? armConfig.positionTolerance : DEFAULT_TOLERANCE;
        return atAngle(setpoint, tolerance);
    }

    /** Returns {@code true} when the arm is within {@code tolerance} of {@code target}. */
    public boolean atAngle(Rotation2d target, Rotation2d tolerance) {
        return Math.abs(getAngle().getDegrees() - target.getDegrees()) <= tolerance.getDegrees();
    }

    // ── Characterization ──────────────────────────────────────────────────────

    /**
     * Ramps open-loop voltage at the arm's {@code horizontalZero} angle until it holds
     * position. The measured holding voltage is {@code kG}.
     */
    public Command gravityCharacterization() {
        Rotation2d horizontalZero = new Rotation2d(motor.getHorizontalZeroRad());
        return Commands.sequence(
            runWithProfile(horizontalZero).until(this::atAngle).withTimeout(3.0),
            voltageRampCommand(3.0, 0.01, 0.05, 10, kg -> {
                Logger.recordOutput(logPrefix + "GravityChar/KgEstimate", kg);
                Logger.recordOutput(logPrefix + "GravityChar/Complete", true);
            }).withTimeout(10.0)
        ).withName("Arm GravityCharacterization");
    }

    @Override
    public void simIterate() {
        if (armSim == null) return;
        if (Logger.hasReplaySource()) return;

        armSim.setInputVoltage(motor.getSimVoltage());
        armSim.update(0.020);

        motor.simUpdate(
                new Rotation2d(armSim.getAngleRads()),
                RadiansPerSecond.of(armSim.getVelocityRadPerSec()));
    }

    @Override
    public void updateTelemetry() {
        processInputs();

        Logger.recordOutput(logPrefix + "AngleDegrees", getAngle().getDegrees());
        Logger.recordOutput(logPrefix + "SetpointDegrees", setpoint.getDegrees());
        Logger.recordOutput(logPrefix + "AtAngle", atAngle());

        Command active = getSubsystem().getCurrentCommand();
        Logger.recordOutput(logPrefix + "ActiveCommand", active != null ? active.getName() : "None");
    }
}
