package com.teamscreamrobotics.motorcontrol;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Turret extends SmartMechanism {

    private static final Angle DEFAULT_TOLERANCE = Degrees.of(1.0);

    private final TurretConfig turretConfig;
    private final DCMotorSim turretSim;

    // CRT encoders — non-null only in CRT mode
    private final CANcoder primaryCRTEncoder;
    private final CANcoder secondaryCRTEncoder;

    private Angle setpoint;
    private double lagCorrectionDeg = 0.0;
    private boolean driftWarningFired = false;

    public Turret(TurretConfig turretConfig) {
        super(turretConfig.motor, turretConfig.resolveLogPrefix());
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

            Angle absolutePosition = computeCRTPosition(
                    phi_A, crt.primaryTurnsPerMechanismTurn(),
                    phi_B, crt.secondaryTurnsPerMechanismTurn());
            motor.resetEncoder(absolutePosition);
            this.setpoint = absolutePosition;
        } else {
            primaryCRTEncoder = null;
            secondaryCRTEncoder = null;
            motor.resetEncoder(turretConfig.startingPosition);
        }

        if (RobotBase.isSimulation() && config.motorModel != null) {
            turretSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            config.motorModel,
                            turretConfig.moiKgMetersSquared,
                            config.gearing),
                    config.motorModel);
        } else {
            turretSim = null;
        }
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    public Command run(Angle angle) {
        return Commands.run(() -> setAngle(angle), config.subsystem)
                .withName("Turret.run(" + angle.in(Degrees) + " deg)");
    }

    public Command runTo(Angle angle) {
        return Commands.run(() -> setAngle(angle), config.subsystem)
                .until(this::atAngle)
                .withName("Turret.runTo(" + angle.in(Degrees) + " deg)");
    }

    /**
     * Continuously tracks a dynamic target angle, automatically wrapping around hard limits
     * to stay on target. When the target angle is outside {@code [hardLimitMin, hardLimitMax]},
     * the nearest equivalent in-range angle is commanded instead.
     *
     * <p>Lag compensation is applied to each update if configured.
     *
     * <p>Requires {@code hardLimitMin} and {@code hardLimitMax} to be set in config.
     * If limits are not configured, behaves identically to {@code run(Supplier<Angle>)}.
     */
    public Command track(Supplier<Angle> targetSupplier) {
        return Commands.run(() -> setAngle(normalizeAngle(targetSupplier.get())), config.subsystem)
                .withName("Turret.track(dynamic)");
    }

    // ── Control ───────────────────────────────────────────────────────────────

    /**
     * Commands the turret to the given angle with optional lag compensation applied.
     * The stored setpoint (used for {@link #atAngle()}) is always the uncompensated target.
     */
    public void setAngle(Angle angle) {
        this.setpoint = angle;
        Angle target = angle;

        if (turretConfig.drivetrainAngularVelocitySupplier != null) {
            double omegaRadPerSec = turretConfig.drivetrainAngularVelocitySupplier.get()
                    .in(RadiansPerSecond);
            double lagCorrectionRad = -omegaRadPerSec * turretConfig.lagCompensationSeconds;
            target = Radians.of(target.in(Radians) + lagCorrectionRad);
            lagCorrectionDeg = Math.toDegrees(lagCorrectionRad);
        }

        motor.setPosition(target);
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

    // ── Simulation ────────────────────────────────────────────────────────────

    @Override
    public void simIterate() {
        if (turretSim == null) return;
        if (Logger.hasReplaySource()) return;

        turretSim.setInputVoltage(motor.getSimVoltage());
        turretSim.update(0.020);

        motor.simUpdate(
                Rotations.of(turretSim.getAngularPositionRotations()),
                RotationsPerSecond.of(turretSim.getAngularVelocityRPM() / 60.0));
    }

    // ── Telemetry ─────────────────────────────────────────────────────────────

    @Override
    public void updateTelemetry() {
        processInputs();

        Logger.recordOutput(logPrefix + "AngleDegrees", getAngle().in(Degrees));
        Logger.recordOutput(logPrefix + "SetpointDegrees", setpoint.in(Degrees));
        Logger.recordOutput(logPrefix + "AtAngle", atAngle());
        Logger.recordOutput(logPrefix + "LagCorrectionDegrees", lagCorrectionDeg);
        Logger.recordOutput(logPrefix + "AbsolutePositionMode",
                primaryCRTEncoder != null ? "CRT" : "Single");

        if (primaryCRTEncoder != null) {
            logCRTDrift();
        }

        Command active = config.subsystem.getCurrentCommand();
        Logger.recordOutput(logPrefix + "ActiveCommand", active != null ? active.getName() : "None");
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /**
     * Normalizes {@code target} to {@code [hardLimitMin, hardLimitMax)} by wrapping.
     * If no hard limits are configured, returns the target unchanged.
     */
    private Angle normalizeAngle(Angle target) {
        if (turretConfig.hardLimitMin == null || turretConfig.hardLimitMax == null) return target;
        double minDeg = turretConfig.hardLimitMin.in(Degrees);
        double maxDeg = turretConfig.hardLimitMax.in(Degrees);
        double rangeDeg = maxDeg - minDeg;
        double t = target.in(Degrees);
        t = ((t - minDeg) % rangeDeg + rangeDeg) % rangeDeg + minDeg;
        return Degrees.of(t);
    }

    private void logCRTDrift() {
        TurretConfig.CRTConfig crt = turretConfig.crtConfig;
        double phi_A = normalizeEncoderReading(
                primaryCRTEncoder.getAbsolutePosition().getValueAsDouble());
        double phi_B = normalizeEncoderReading(
                secondaryCRTEncoder.getAbsolutePosition().getValueAsDouble());

        Angle livePosition = computeCRTPosition(
                phi_A, crt.primaryTurnsPerMechanismTurn(),
                phi_B, crt.secondaryTurnsPerMechanismTurn());

        double driftDeg = Math.abs(livePosition.in(Degrees) - getAngle().in(Degrees));

        Logger.recordOutput(logPrefix + "CRT/LiveAbsolutePositionDegrees", livePosition.in(Degrees));
        Logger.recordOutput(logPrefix + "CRT/DriftDegrees", driftDeg);

        if (driftDeg > crt.driftWarningThreshold().in(Degrees) && !driftWarningFired) {
            DriverStation.reportWarning(
                    String.format("Turret CRT position drift: %.1f deg (threshold %.1f deg)",
                            driftDeg, crt.driftWarningThreshold().in(Degrees)),
                    false);
            driftWarningFired = true;
        } else if (driftDeg <= crt.driftWarningThreshold().in(Degrees)) {
            driftWarningFired = false;
        }
    }

    /**
     * Converts a raw Phoenix 6 absolute-position reading from [-0.5, 0.5) to [0, 1).
     */
    private static double normalizeEncoderReading(double rawRotations) {
        return rawRotations < 0 ? rawRotations + 1.0 : rawRotations;
    }

    /**
     * Computes the mechanism's absolute position from two encoder readings using the
     * Chinese Remainder Theorem.
     *
     * <p>Algorithm:
     * <ol>
     *   <li>Convert each encoder reading to mechanism rotations: {@code theta = phi / turnsPerMechTurn}
     *   <li>Find how many full primary-range cycles separate the coarse and fine estimates
     *   <li>Return the fine estimate offset by that many cycles
     * </ol>
     *
     * @param phi_A                        primary encoder reading [0, 1) in encoder rotations
     * @param primaryTurnsPerMechanismTurn encoder turns per mechanism turn (fine encoder)
     * @param phi_B                        secondary encoder reading [0, 1)
     * @param secondaryTurnsPerMechanismTurn encoder turns per mechanism turn (coarse encoder)
     */
    private static Angle computeCRTPosition(
            double phi_A, double primaryTurnsPerMechanismTurn,
            double phi_B, double secondaryTurnsPerMechanismTurn) {
        double thetaFine   = phi_A / primaryTurnsPerMechanismTurn;
        double thetaCoarse = phi_B / secondaryTurnsPerMechanismTurn;
        double primaryRange = 1.0 / primaryTurnsPerMechanismTurn;
        double k = Math.round((thetaCoarse - thetaFine) / primaryRange);
        return Rotations.of(thetaFine + k * primaryRange);
    }
}
