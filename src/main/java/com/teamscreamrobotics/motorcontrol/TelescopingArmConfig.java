package com.teamscreamrobotics.motorcontrol;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.power.PowerPriority;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;

/**
 * Configuration for a telescoping arm that combines a rotational pivot with a
 * linear extension.
 *
 * <p>
 * Motors are added via {@link #withPivotMotor} and {@link #withExtensionMotor},
 * which lazily
 * create {@link TalonFXWrapper} instances. All {@code withPivot*} and
 * {@code withExtension*}
 * methods delegate to internal {@link PivotConfig} and {@link ElevatorConfig}
 * instances.
 *
 * <h2>Extension coordinate convention</h2>
 * <p>
 * All extension distances represent the absolute radial distance from the pivot
 * axis to the
 * end effector. {@link #withExtensionLimits} sets both hard stop bounds; the
 * encoder is seeded
 * to {@link #withStartingExtension} on construction.
 *
 * <h2>Mechanism circumference requirement</h2>
 * <p>
 * Call {@link #withExtensionMechanismCircumference} before
 * {@link #withExtensionSoftLimits}.
 */
public class TelescopingArmConfig {

    final SubsystemBase subsystem;

    // Internal configs — created lazily by withPivotMotor / withExtensionMotor
    PivotConfig pivotConfig;
    ElevatorConfig elevatorConfig;

    // TelescopingArm-level fields
    double pivotGravityKg = 0.0;
    double maxTaskVelocityMps = 1.0;
    double maxTaskAccelMps2 = 2.0;
    String logPrefix = null;

    public TelescopingArmConfig(SubsystemBase subsystem) {
        this.subsystem = subsystem;
    }

    // ── Motor registration ────────────────────────────────────────────────────

    /**
     * Registers the pivot motor and creates an internal {@link PivotConfig} for it.
     */
    public TelescopingArmConfig withPivotMotor(TalonFX motor, DCMotor motorModel,
            FollowerConfig... followers) {
        TalonFXWrapper wrapper = new TalonFXWrapper(motor, motorModel, subsystem, followers);
        this.pivotConfig = new PivotConfig(wrapper);
        return this;
    }

    /**
     * Registers the extension motor and creates an internal {@link ElevatorConfig}
     * for it.
     */
    public TelescopingArmConfig withExtensionMotor(TalonFX motor, DCMotor motorModel,
            FollowerConfig... followers) {
        TalonFXWrapper wrapper = new TalonFXWrapper(motor, motorModel, subsystem, followers);
        this.elevatorConfig = new ElevatorConfig(wrapper);
        return this;
    }

    // ── Top-level arm config ──────────────────────────────────────────────────

    /**
     * Sets the pivot gravity feedforward constant — volts required to hold the arm
     * horizontal at full extension.
     */
    public TelescopingArmConfig withPivotGravity(double kGVolts) {
        this.pivotGravityKg = kGVolts;
        return this;
    }

    /**
     * Sets the task-space trapezoidal profile constraints for end-effector
     * trajectory planning.
     */
    public TelescopingArmConfig withTaskSpaceConstraints(double maxVelocityMps, double maxAccelMps2) {
        this.maxTaskVelocityMps = maxVelocityMps;
        this.maxTaskAccelMps2 = maxAccelMps2;
        return this;
    }

    /** Overrides the default AdvantageKit log prefix for this mechanism. */
    public TelescopingArmConfig withLogPrefix(String logPrefix) {
        this.logPrefix = logPrefix;
        return this;
    }

    // ── Pivot configuration ───────────────────────────────────────────────────

    /** @see PivotConfig#withGearing(double) */
    public TelescopingArmConfig withPivotGearing(double reduction) {
        pivotConfig.withGearing(reduction);
        return this;
    }

    public TelescopingArmConfig withPivotMOI(double moiKgMetersSquared) {
        pivotConfig.moiKgMetersSquared = moiKgMetersSquared;
        return this;
    }

    public TelescopingArmConfig withPivotMOI(Distance radius, Mass mass) {
        pivotConfig.moiKgMetersSquared = 0.5 * mass.in(Kilograms) * Math.pow(radius.in(Meters), 2);
        return this;
    }

    public TelescopingArmConfig withPivotLimits(Rotation2d min, Rotation2d max) {
        pivotConfig.withHardLimit(min, max);
        return this;
    }

    public TelescopingArmConfig withPivotSoftLimits(Rotation2d min, Rotation2d max) {
        pivotConfig.withSoftLimits(min, max);
        return this;
    }

    public TelescopingArmConfig withStartingAngle(Rotation2d angle) {
        pivotConfig.withStartingPosition(angle);
        return this;
    }

    public TelescopingArmConfig withPivotClosedLoopController(double kP, double kI, double kD) {
        pivotConfig.withClosedLoopController(kP, kI, kD);
        return this;
    }

    public TelescopingArmConfig withPivotClosedLoopController(double kP, double kI, double kD,
            AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
        pivotConfig.withClosedLoopController(kP, kI, kD, maxVelocity, maxAcceleration);
        return this;
    }

    public TelescopingArmConfig withPivotSimClosedLoopController(double kP, double kI, double kD,
            AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
        pivotConfig.withSimClosedLoopController(kP, kI, kD, maxVelocity, maxAcceleration);
        return this;
    }

    public TelescopingArmConfig withPivotFeedforward(ArmFeedforward ff) {
        pivotConfig.withFeedforward(ff);
        return this;
    }

    public TelescopingArmConfig withPivotSimFeedforward(ArmFeedforward ff) {
        pivotConfig.withSimFeedforward(ff);
        return this;
    }

    public TelescopingArmConfig withPivotNeutralMode(NeutralModeValue mode) {
        pivotConfig.withNeutralMode(mode);
        return this;
    }

    public TelescopingArmConfig withPivotInverted(InvertedValue invert) {
        pivotConfig.withInverted(invert);
        return this;
    }

    public TelescopingArmConfig withPivotStatorCurrentLimit(Current limit) {
        pivotConfig.withStatorCurrentLimit(limit);
        return this;
    }

    public TelescopingArmConfig withPivotSupplyCurrentLimit(Current limit) {
        pivotConfig.withSupplyCurrentLimit(limit);
        return this;
    }

    public TelescopingArmConfig withPivotPositionTolerance(Rotation2d tolerance) {
        pivotConfig.withPositionTolerance(tolerance);
        return this;
    }

    public TelescopingArmConfig withPivotPowerPriority(PowerPriority priority) {
        pivotConfig.withPowerPriority(priority);
        return this;
    }

    public TelescopingArmConfig withPivotCANcoder(int canId) {
        pivotConfig.withCANcoder(canId);
        return this;
    }

    public TelescopingArmConfig withPivotCANcoder(int canId, String canbus) {
        pivotConfig.withCANcoder(canId, canbus);
        return this;
    }

    public TelescopingArmConfig withPivotCANcoderOffset(double offsetRotations) {
        pivotConfig.withCANcoderOffset(offsetRotations);
        return this;
    }

    public TelescopingArmConfig withPivotExtraConfigs(Consumer<TalonFXConfiguration> consumer) {
        pivotConfig.withExtraConfigs(consumer);
        return this;
    }

    public TelescopingArmConfig withPivotExtraSimConfigs(Consumer<TalonFXConfiguration> consumer) {
        pivotConfig.withExtraSimConfigs(consumer);
        return this;
    }

    public TelescopingArmConfig withPivotExtraCANcoderConfigs(Consumer<CANcoderConfiguration> consumer) {
        pivotConfig.withExtraCANcoderConfigs(consumer);
        return this;
    }

    // ── Extension configuration ───────────────────────────────────────────────

    public TelescopingArmConfig withExtensionGearing(double reduction) {
        elevatorConfig.withGearing(reduction);
        return this;
    }

    /**
     * Sets the drum or sprocket circumference. Must be called before
     * {@link #withExtensionSoftLimits}.
     */
    public TelescopingArmConfig withExtensionMechanismCircumference(Distance circumference) {
        elevatorConfig.withMechanismCircumference(circumference);
        return this;
    }

    public TelescopingArmConfig withExtensionLimits(Distance min, Distance max) {
        elevatorConfig.withHardLimits(min, max);
        return this;
    }

    public TelescopingArmConfig withExtensionSoftLimits(Distance min, Distance max) {
        elevatorConfig.withSoftLimits(min, max);
        return this;
    }

    public TelescopingArmConfig withStartingExtension(Distance extension) {
        elevatorConfig.withStartingHeight(extension);
        return this;
    }

    public TelescopingArmConfig withExtensionMass(Mass mass) {
        elevatorConfig.withMass(mass);
        return this;
    }

    public TelescopingArmConfig withExtensionClosedLoopController(double kP, double kI, double kD) {
        elevatorConfig.withClosedLoopController(kP, kI, kD);
        return this;
    }

    public TelescopingArmConfig withExtensionClosedLoopController(double kP, double kI, double kD,
            AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
        elevatorConfig.withClosedLoopController(kP, kI, kD, maxVelocity, maxAcceleration);
        return this;
    }

    public TelescopingArmConfig withExtensionSimClosedLoopController(double kP, double kI, double kD,
            AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
        elevatorConfig.withSimClosedLoopController(kP, kI, kD, maxVelocity, maxAcceleration);
        return this;
    }

    public TelescopingArmConfig withExtensionFeedforward(ElevatorFeedforward ff) {
        elevatorConfig.withFeedforward(ff);
        return this;
    }

    public TelescopingArmConfig withExtensionSimFeedforward(ElevatorFeedforward ff) {
        elevatorConfig.withSimFeedforward(ff);
        return this;
    }

    public TelescopingArmConfig withExtensionNeutralMode(NeutralModeValue mode) {
        elevatorConfig.withNeutralMode(mode);
        return this;
    }

    public TelescopingArmConfig withExtensionInverted(InvertedValue invert) {
        elevatorConfig.withInverted(invert);
        return this;
    }

    public TelescopingArmConfig withExtensionStatorCurrentLimit(Current limit) {
        elevatorConfig.withStatorCurrentLimit(limit);
        return this;
    }

    public TelescopingArmConfig withExtensionSupplyCurrentLimit(Current limit) {
        elevatorConfig.withSupplyCurrentLimit(limit);
        return this;
    }

    public TelescopingArmConfig withExtensionPositionTolerance(Distance tolerance) {
        elevatorConfig.withPositionTolerance(tolerance);
        return this;
    }

    public TelescopingArmConfig withExtensionPowerPriority(PowerPriority priority) {
        elevatorConfig.withPowerPriority(priority);
        return this;
    }

    public TelescopingArmConfig withExtensionExtraConfigs(Consumer<TalonFXConfiguration> consumer) {
        elevatorConfig.withExtraConfigs(consumer);
        return this;
    }

    public TelescopingArmConfig withExtensionExtraSimConfigs(Consumer<TalonFXConfiguration> consumer) {
        elevatorConfig.withExtraSimConfigs(consumer);
        return this;
    }

    // ── Logging ───────────────────────────────────────────────────────────────

    public String resolveLogPrefix() {
        if (logPrefix != null)
            return logPrefix;
        return "Mechanisms/TelescopingArm/" + subsystem.getClass().getSimpleName() + "/";
    }
}
