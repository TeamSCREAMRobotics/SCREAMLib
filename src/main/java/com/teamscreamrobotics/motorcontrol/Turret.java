package com.teamscreamrobotics.motorcontrol;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

/**
 * Turret mechanism driven by a TalonFX with optional CRT absolute positioning and
 * drivetrain-lag compensation.
 *
 * <p>Supports two encoder modes:
 * <ul>
 *   <li><b>Single encoder</b> — motor encoder only; position seeded to {@code startingPosition}
 *       on construction.</li>
 *   <li><b>CRT mode</b> — two CANcoders compute an absolute position at startup via the
 *       Chinese Remainder Theorem; ongoing drift is logged and warned.</li>
 * </ul>
 */
public class Turret extends SmartMechanism {

    private static final Rotation2d DEFAULT_TOLERANCE = Rotation2d.fromDegrees(1.0);

    private final TurretConfig turretConfig;
    private final DCMotorSim turretSim;

    // CRT encoders — non-null only in CRT mode
    private final CANcoder primaryCRTEncoder;
    private final CANcoder secondaryCRTEncoder;

    private Rotation2d setpoint;
    private double lagCorrectionDeg = 0.0;
    private double lastDriftWarningTime = -10.0;

    /** Constructs the turret, applies motor config, and seeds the encoder (or reads CRT position). */
    public Turret(TurretConfig turretConfig) {
        super(turretConfig.motor, turretConfig.resolveLogPrefix());
        turretConfig.applyBuilt();
        this.turretConfig = turretConfig;
        this.setpoint = turretConfig.startingPosition;

        if (turretConfig.crtConfig != null) {
            TurretConfig.CRTConfig crt = turretConfig.crtConfig;

            primaryCRTEncoder = crt.primaryCanbus().isEmpty()
                    ? new CANcoder(crt.primaryCanId())
                    : new CANcoder(crt.primaryCanId(), crt.primaryCanbus());
            secondaryCRTEncoder = crt.secondaryCanbus().isEmpty()
                    ? new CANcoder(crt.secondaryCanId())
                    : new CANcoder(crt.secondaryCanId(), crt.secondaryCanbus());

            CANcoderConfiguration primaryCfg = new CANcoderConfiguration();
            primaryCfg.MagnetSensor = new MagnetSensorConfigs()
                    .withMagnetOffset(crt.primaryMagnetOffsetRotations());
            primaryCRTEncoder.getConfigurator().apply(primaryCfg);

            CANcoderConfiguration secondaryCfg = new CANcoderConfiguration();
            secondaryCfg.MagnetSensor = new MagnetSensorConfigs()
                    .withMagnetOffset(crt.secondaryMagnetOffsetRotations());
            secondaryCRTEncoder.getConfigurator().apply(secondaryCfg);

            primaryCRTEncoder.getAbsolutePosition().setUpdateFrequency(50);
            secondaryCRTEncoder.getAbsolutePosition().setUpdateFrequency(50);
            primaryCRTEncoder.optimizeBusUtilization();
            secondaryCRTEncoder.optimizeBusUtilization();

            double phi_A = normalizeEncoderReading(
                    primaryCRTEncoder.getAbsolutePosition().refresh().getValueAsDouble());
            double phi_B = normalizeEncoderReading(
                    secondaryCRTEncoder.getAbsolutePosition().refresh().getValueAsDouble());

            Rotation2d absolutePosition = computeCRTPosition(
                    phi_A, crt.primaryTurnsPerMechanismTurn(),
                    phi_B, crt.secondaryTurnsPerMechanismTurn());
            motor.resetEncoder(absolutePosition);
            this.setpoint = absolutePosition;
        } else {
            primaryCRTEncoder = null;
            secondaryCRTEncoder = null;
            motor.resetEncoder(turretConfig.startingPosition);
        }

        if (RobotBase.isSimulation() && turretConfig.motorModel != null) {
            turretSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            turretConfig.motorModel,
                            turretConfig.moiKgMetersSquared,
                            motor.getRuntimeInfo().gearing()),
                    turretConfig.motorModel);
        } else {
            turretSim = null;
        }
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    /**
     * Runs the turret to {@code angle} continuously using MotionMagic profiled position control.
     * The command never finishes on its own; use {@link #runToWithProfile} for a one-shot move.
     */
    public Command runWithProfile(Rotation2d angle) {
        return Commands.run(() -> setAngleWithProfile(angle), getSubsystem())
                .withName("Turret.runWithProfile(" + angle.getDegrees() + " deg)");
    }

    /** Runs the turret to {@code angle} via MotionMagic and finishes once {@link #atAngle()} is true. */
    public Command runToWithProfile(Rotation2d angle) {
        return Commands.run(() -> setAngleWithProfile(angle), getSubsystem())
                .until(this::atAngle)
                .withName("Turret.runToWithProfile(" + angle.getDegrees() + " deg)");
    }

    /**
     * Runs the turret to {@code angle} continuously using direct position control (no profile).
     * The command never finishes on its own; use {@link #runTo} for a one-shot move.
     */
    public Command run(Rotation2d angle) {
        return Commands.run(() -> setAngle(angle), getSubsystem())
                .withName("Turret.run(" + angle.getDegrees() + " deg)");
    }

    /** Runs the turret to {@code angle} via direct position control and finishes once {@link #atAngle()} is true. */
    public Command runTo(Rotation2d angle) {
        return Commands.run(() -> setAngle(angle), getSubsystem())
                .until(this::atAngle)
                .withName("Turret.runTo(" + angle.getDegrees() + " deg)");
    }

    /**
     * Continuously tracks a dynamic target angle, automatically wrapping around hard limits
     * to stay on target.
     */
    public Command track(Supplier<Rotation2d> targetSupplier) {
        return Commands.run(() -> setAngle(normalizeAngle(targetSupplier.get())), getSubsystem())
                .withName("Turret.track(dynamic)");
    }

    // ── Control ───────────────────────────────────────────────────────────────

    /** Sends a MotionMagic profiled position setpoint, applying lag compensation if configured. */
    public void setAngleWithProfile(Rotation2d angle) {
        this.setpoint = angle;
        Rotation2d target = angle;

        if (turretConfig.drivetrainAngularVelocitySupplier != null) {
            double omegaRadPerSec = turretConfig.drivetrainAngularVelocitySupplier.get()
                    .in(RadiansPerSecond);
            double lagCorrectionRad = -omegaRadPerSec * turretConfig.lagCompensationSeconds;
            target = new Rotation2d(target.getRadians() + lagCorrectionRad);
            lagCorrectionDeg = Math.toDegrees(lagCorrectionRad);
        }

        motor.setPositionProfiled(target);
    }

    /** Sends a direct position setpoint (no profile), applying lag compensation if configured. */
    public void setAngle(Rotation2d angle) {
        this.setpoint = angle;
        Rotation2d target = angle;

        if (turretConfig.drivetrainAngularVelocitySupplier != null) {
            double omegaRadPerSec = turretConfig.drivetrainAngularVelocitySupplier.get()
                    .in(RadiansPerSecond);
            double lagCorrectionRad = -omegaRadPerSec * turretConfig.lagCompensationSeconds;
            target = new Rotation2d(target.getRadians() + lagCorrectionRad);
            lagCorrectionDeg = Math.toDegrees(lagCorrectionRad);
        }

        motor.setPosition(target);
    }

    /** Returns the current mechanism angle from the motor's feedback sensor. */
    public Rotation2d getAngle() {
        return motor.getMechanismPosition();
    }

    /** Returns {@code true} when the turret is within tolerance of the most recent setpoint. */
    public boolean atAngle() {
        Rotation2d tolerance = turretConfig.positionTolerance != null
                ? turretConfig.positionTolerance : DEFAULT_TOLERANCE;
        return atAngle(setpoint, tolerance);
    }

    /** Returns {@code true} when the turret is within {@code tolerance} of {@code target}. */
    public boolean atAngle(Rotation2d target, Rotation2d tolerance) {
        return Math.abs(getAngle().getDegrees() - target.getDegrees()) <= tolerance.getDegrees();
    }

    // ── Simulation ────────────────────────────────────────────────────────────

    @Override
    public void simIterate() {
        if (turretSim == null) return;
        if (Logger.hasReplaySource()) return;

        turretSim.setInputVoltage(motor.getSimVoltage());
        turretSim.update(0.020);

        motor.simUpdate(
                Rotation2d.fromRotations(turretSim.getAngularPositionRotations()),
                RotationsPerSecond.of(turretSim.getAngularVelocityRPM() / 60.0));
    }

    // ── Telemetry ─────────────────────────────────────────────────────────────

    @Override
    public void updateTelemetry() {
        processInputs();

        Logger.recordOutput(logPrefix + "AngleDegrees", getAngle().getDegrees());
        Logger.recordOutput(logPrefix + "SetpointDegrees", setpoint.getDegrees());
        Logger.recordOutput(logPrefix + "AtAngle", atAngle());
        Logger.recordOutput(logPrefix + "LagCorrectionDegrees", lagCorrectionDeg);
        Logger.recordOutput(logPrefix + "AbsolutePositionMode",
                primaryCRTEncoder != null ? "CRT" : "Single");

        if (primaryCRTEncoder != null) {
            logCRTDrift();
        }

        Command active = getSubsystem().getCurrentCommand();
        Logger.recordOutput(logPrefix + "ActiveCommand", active != null ? active.getName() : "None");
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    private Rotation2d normalizeAngle(Rotation2d target) {
        if (turretConfig.hardLimitMin == null || turretConfig.hardLimitMax == null) return target;
        double minDeg = turretConfig.hardLimitMin.getDegrees();
        double maxDeg = turretConfig.hardLimitMax.getDegrees();
        double rangeDeg = maxDeg - minDeg;
        double t = target.getDegrees();
        t = ((t - minDeg) % rangeDeg + rangeDeg) % rangeDeg + minDeg;
        return Rotation2d.fromDegrees(t);
    }

    private void logCRTDrift() {
        TurretConfig.CRTConfig crt = turretConfig.crtConfig;
        double phi_A = normalizeEncoderReading(
                primaryCRTEncoder.getAbsolutePosition().getValueAsDouble());
        double phi_B = normalizeEncoderReading(
                secondaryCRTEncoder.getAbsolutePosition().getValueAsDouble());

        Rotation2d livePosition = computeCRTPosition(
                phi_A, crt.primaryTurnsPerMechanismTurn(),
                phi_B, crt.secondaryTurnsPerMechanismTurn());

        double driftDeg = Math.abs(livePosition.getDegrees() - getAngle().getDegrees());

        Logger.recordOutput(logPrefix + "CRT/LiveAbsolutePositionDegrees", livePosition.getDegrees());
        Logger.recordOutput(logPrefix + "CRT/DriftDegrees", driftDeg);

        if (driftDeg > crt.driftWarningThreshold().getDegrees()
                && Timer.getFPGATimestamp() - lastDriftWarningTime >= 10.0) {
            DriverStation.reportWarning(
                    String.format("Turret CRT position drift: %.1f deg (threshold %.1f deg)",
                            driftDeg, crt.driftWarningThreshold().getDegrees()),
                    false);
            lastDriftWarningTime = Timer.getFPGATimestamp();
        }
    }

    private static double normalizeEncoderReading(double rawRotations) {
        return rawRotations < 0 ? rawRotations + 1.0 : rawRotations;
    }

    private static Rotation2d computeCRTPosition(
            double phi_A, double primaryTurnsPerMechanismTurn,
            double phi_B, double secondaryTurnsPerMechanismTurn) {
        double thetaFine   = phi_A / primaryTurnsPerMechanismTurn;
        double thetaCoarse = phi_B / secondaryTurnsPerMechanismTurn;
        double primaryRange = 1.0 / primaryTurnsPerMechanismTurn;
        double k = Math.round((thetaCoarse - thetaFine) / primaryRange);
        return Rotation2d.fromRotations(thetaFine + k * primaryRange);
    }
}
