package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class TurretConfig {

    public final SmartMotorController motor;

    public Angle startingPosition = Degrees.of(0);
    /** Physical hard stops. Also used by {@link Turret#track} to normalize the target into range. */
    public Angle hardLimitMin = null;
    public Angle hardLimitMax = null;
    public double moiKgMetersSquared = 0.001;
    public String logPrefix = null;

    /**
     * Dual-encoder absolute positioning via the Chinese Remainder Theorem.
     * If null, the motor's own CANcoder config ({@code motor.getConfig().cancoder} with
     * {@code useAsAbsolutePosition=true}) is used for absolute seeding instead.
     */
    public CRTConfig crtConfig = null;

    /** Optional lag compensation. See {@link #withLagCompensation}. */
    public Supplier<AngularVelocity> drivetrainAngularVelocitySupplier = null;
    public double lagCompensationSeconds = 0.0;

    // ── CRT config ────────────────────────────────────────────────────────────

    /**
     * Parameters for two-CANcoder Chinese Remainder Theorem absolute positioning.
     *
     * <p>The primary encoder should be the <b>fine</b> (faster-turning) one. The secondary
     * should be the <b>coarse</b> (slower-turning) one. The combined unique range is:
     * <pre>
     *   range = (1 / primaryTurnsPerMechanismTurn) * (1 / secondaryTurnsPerMechanismTurn)
     * </pre>
     * This is maximized when the two ranges are coprime integers.
     *
     * <p>Example: primary geared 1:1 ({@code primaryTurnsPerMechanismTurn = 1.0}) and secondary
     * geared 1:15 ({@code secondaryTurnsPerMechanismTurn = 1.0/15.0}) → 15 mechanism rotations
     * of unique range with 1-rotation resolution.
     *
     * <p><b>Do not</b> set {@code useAsAbsolutePosition=true} on any CANcoder in the motor
     * config when using CRT mode — the Turret constructor performs the seeding manually.
     */
    public record CRTConfig(
            int primaryCanId,
            String primaryCanbus,
            double primaryMagnetOffsetRotations,
            double primaryTurnsPerMechanismTurn,
            int secondaryCanId,
            String secondaryCanbus,
            double secondaryMagnetOffsetRotations,
            double secondaryTurnsPerMechanismTurn,
            Angle driftWarningThreshold
    ) {
        /** Convenience overload: default CAN bus, 5-degree drift warning threshold. */
        public CRTConfig(
                int primaryCanId, double primaryMagnetOffsetRotations, double primaryTurnsPerMechanismTurn,
                int secondaryCanId, double secondaryMagnetOffsetRotations, double secondaryTurnsPerMechanismTurn) {
            this(primaryCanId, "", primaryMagnetOffsetRotations, primaryTurnsPerMechanismTurn,
                 secondaryCanId, "", secondaryMagnetOffsetRotations, secondaryTurnsPerMechanismTurn,
                 Degrees.of(5.0));
        }
    }

    // ── Constructor ───────────────────────────────────────────────────────────

    public TurretConfig(SmartMotorController motor) {
        this.motor = motor;
    }

    // ── Fluent builders ───────────────────────────────────────────────────────

    public TurretConfig withStartingPosition(Angle startingPosition) {
        this.startingPosition = startingPosition;
        return this;
    }

    /** Sets physical hard-stop angles. The motor's soft limits should be configured separately. */
    public TurretConfig withHardLimit(Angle min, Angle max) {
        this.hardLimitMin = min;
        this.hardLimitMax = max;
        return this;
    }

    /** Sets motor-level soft limits and applies them to the hardware. */
    public TurretConfig withSoftLimits(Angle min, Angle max) {
        motor.getConfig().withSoftLimit(min, max);
        motor.reconfigure();
        return this;
    }

    /**
     * Enables continuous wrap on the motor (for full-rotation turrets with no hard stops).
     * Also stores the wrap range as hard limits so {@link Turret#track} can normalize into it.
     */
    public TurretConfig withWrapping(Angle min, Angle max) {
        this.hardLimitMin = min;
        this.hardLimitMax = max;
        motor.getConfig().withContinuousWrap(true);
        motor.reconfigure();
        return this;
    }

    public TurretConfig withMOI(Distance radius, Mass mass) {
        this.moiKgMetersSquared = 0.5 * mass.in(Kilograms) * Math.pow(radius.in(Meters), 2);
        return this;
    }

    public TurretConfig withCRTPositioning(CRTConfig crtConfig) {
        this.crtConfig = crtConfig;
        return this;
    }

    /**
     * Enables heading-lag feedforward compensation.
     *
     * <p>When the drivetrain is rotating, the turret's robot-frame setpoint must lead the
     * drivetrain rotation by {@code lagSeconds} to maintain accurate field-frame targeting.
     * The applied correction is:
     * <pre>
     *   correctedAngle = desiredAngle - drivetrainOmega * lagSeconds
     * </pre>
     * where omega is positive counterclockwise (WPILib convention).
     *
     * @param drivetrainAngularVelocitySupplier field-relative drivetrain angular velocity
     * @param lagSeconds                        measured control loop lag (typically 0.02–0.10 s)
     */
    public TurretConfig withLagCompensation(
            Supplier<AngularVelocity> drivetrainAngularVelocitySupplier,
            double lagSeconds) {
        this.drivetrainAngularVelocitySupplier = drivetrainAngularVelocitySupplier;
        this.lagCompensationSeconds = lagSeconds;
        return this;
    }

    public TurretConfig withLogPrefix(String logPrefix) {
        this.logPrefix = logPrefix;
        return this;
    }

    public String resolveLogPrefix() {
        if (logPrefix != null) return logPrefix;
        return "Mechanisms/Turret/" + motor.getConfig().subsystem.getClass().getSimpleName() + "/";
    }
}
