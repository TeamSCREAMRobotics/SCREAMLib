package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

/**
 * Rotational pivot mechanism driven by a TalonFX.
 *
 * <p>Identical control surface to {@link Arm} but uses a {@link DCMotorSim} (flywheel-style)
 * for simulation rather than {@link edu.wpi.first.wpilibj.simulation.SingleJointedArmSim}.
 * Intended for mechanisms where gravity effects are handled via feedforward rather than
 * the simulator model (e.g., wrists, turrets mounted on moving structures).
 */
public class Pivot extends SmartMechanism {

    private static final Rotation2d DEFAULT_TOLERANCE = Rotation2d.fromDegrees(1.0);

    private final PivotConfig pivotConfig;
    private final DCMotorSim pivotSim;

    private Rotation2d setpoint;

    /** Constructs the pivot, applies motor config, and seeds the encoder to the starting position. */
    public Pivot(PivotConfig pivotConfig) {
        super(pivotConfig.motor, pivotConfig.resolveLogPrefix());
        pivotConfig.applyBuilt();
        this.pivotConfig = pivotConfig;
        this.setpoint = pivotConfig.startingPosition;

        motor.resetEncoder(pivotConfig.startingPosition);

        if (RobotBase.isSimulation() && pivotConfig.motorModel != null) {
            pivotSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            pivotConfig.motorModel,
                            pivotConfig.moiKgMetersSquared,
                            motor.getRuntimeInfo().gearing()),
                    pivotConfig.motorModel);
        } else {
            pivotSim = null;
        }
    }

    /**
     * Runs the pivot to {@code angle} continuously using MotionMagic profiled position control.
     * The command never finishes on its own; use {@link #runToWithProfile} for a one-shot move.
     */
    public Command runWithProfile(Rotation2d angle) {
        return Commands.run(() -> setAngleWithProfile(angle), getSubsystem())
                .withName("Pivot.runWithProfile(" + angle.getDegrees() + " deg)");
    }

    /** Runs the pivot to {@code angle} via MotionMagic and finishes once {@link #atAngle()} is true. */
    public Command runToWithProfile(Rotation2d angle) {
        return Commands.run(() -> setAngleWithProfile(angle), getSubsystem())
                .until(this::atAngle)
                .withName("Pivot.runToWithProfile(" + angle.getDegrees() + " deg)");
    }

    /**
     * Runs the pivot to {@code angle} continuously using direct position control (no profile).
     * The command never finishes on its own; use {@link #runTo} for a one-shot move.
     */
    public Command run(Rotation2d angle) {
        return Commands.run(() -> setAngle(angle), getSubsystem())
                .withName("Pivot.run(" + angle.getDegrees() + " deg)");
    }

    /** Runs the pivot to {@code angle} via direct position control and finishes once {@link #atAngle()} is true. */
    public Command runTo(Rotation2d angle) {
        return Commands.run(() -> setAngle(angle), getSubsystem())
                .until(this::atAngle)
                .withName("Pivot.runTo(" + angle.getDegrees() + " deg)");
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

    /** Returns {@code true} when the pivot is within tolerance of the most recent setpoint. */
    public boolean atAngle() {
        Rotation2d tolerance = pivotConfig.positionTolerance != null
                ? pivotConfig.positionTolerance : DEFAULT_TOLERANCE;
        return atAngle(setpoint, tolerance);
    }

    /** Returns {@code true} when the pivot is within {@code tolerance} of {@code target}. */
    public boolean atAngle(Rotation2d target, Rotation2d tolerance) {
        return Math.abs(getAngle().getDegrees() - target.getDegrees()) <= tolerance.getDegrees();
    }

    // ── Characterization ──────────────────────────────────────────────────────

    /**
     * Ramps open-loop voltage at the pivot's {@code horizontalZero} angle until it holds
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
        ).withName("Pivot GravityCharacterization");
    }

    @Override
    public void simIterate() {
        if (pivotSim == null) return;
        if (Logger.hasReplaySource()) return;

        pivotSim.setInputVoltage(motor.getSimVoltage());
        pivotSim.update(0.020);

        motor.simUpdate(
                Rotation2d.fromRotations(pivotSim.getAngularPositionRotations()),
                RotationsPerSecond.of(pivotSim.getAngularVelocityRPM() / 60.0));
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
