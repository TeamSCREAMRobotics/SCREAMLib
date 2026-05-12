package com.teamscreamrobotics.motorcontrol;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.power.PowerPriority;

import edu.wpi.first.math.controller.ArmFeedforward;
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
 * Configuration for a telescoping arm with an additional wrist pivot at the end effector.
 *
 * <p>Extends {@link TelescopingArmConfig} with a third motor (the wrist) whose configuration
 * delegates to an internal {@link PivotConfig}. All {@code withWrist*} methods mirror the
 * {@code withPivot*} API.
 */
public class TelescopingArmWithWristConfig extends TelescopingArmConfig {

    PivotConfig wristConfig;

    public TelescopingArmWithWristConfig(SubsystemBase subsystem) {
        super(subsystem);
    }

    // ── Motor registration ────────────────────────────────────────────────────

    /** Registers the wrist pivot motor and creates an internal {@link PivotConfig} for it. */
    public TelescopingArmWithWristConfig withWristMotor(TalonFX motor, DCMotor motorModel,
            FollowerConfig... followers) {
        TalonFXWrapper wrapper = new TalonFXWrapper(motor, motorModel, subsystem, followers);
        this.wristConfig = new PivotConfig(wrapper);
        return this;
    }

    // ── Wrist configuration ───────────────────────────────────────────────────

    public TelescopingArmWithWristConfig withWristGearing(double reduction) {
        wristConfig.withGearing(reduction); return this;
    }

    public TelescopingArmWithWristConfig withWristMOI(double moiKgMetersSquared) {
        wristConfig.moiKgMetersSquared = moiKgMetersSquared; return this;
    }

    public TelescopingArmWithWristConfig withWristMOI(Distance radius, Mass mass) {
        wristConfig.moiKgMetersSquared = 0.5 * mass.in(Kilograms) * Math.pow(radius.in(Meters), 2);
        return this;
    }

    public TelescopingArmWithWristConfig withWristLimits(Rotation2d min, Rotation2d max) {
        wristConfig.withHardLimit(min, max); return this;
    }

    public TelescopingArmWithWristConfig withWristSoftLimits(Rotation2d min, Rotation2d max) {
        wristConfig.withSoftLimits(min, max); return this;
    }

    public TelescopingArmWithWristConfig withStartingWristRotation2d(Rotation2d angle) {
        wristConfig.withStartingPosition(angle); return this;
    }

    public TelescopingArmWithWristConfig withWristClosedLoopController(double kP, double kI, double kD) {
        wristConfig.withClosedLoopController(kP, kI, kD); return this;
    }

    public TelescopingArmWithWristConfig withWristClosedLoopController(double kP, double kI, double kD,
            AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
        wristConfig.withClosedLoopController(kP, kI, kD, maxVelocity, maxAcceleration); return this;
    }

    public TelescopingArmWithWristConfig withWristSimClosedLoopController(double kP, double kI, double kD,
            AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
        wristConfig.withSimClosedLoopController(kP, kI, kD, maxVelocity, maxAcceleration); return this;
    }

    public TelescopingArmWithWristConfig withWristFeedforward(ArmFeedforward ff) {
        wristConfig.withFeedforward(ff); return this;
    }

    public TelescopingArmWithWristConfig withWristSimFeedforward(ArmFeedforward ff) {
        wristConfig.withSimFeedforward(ff); return this;
    }

    public TelescopingArmWithWristConfig withWristNeutralMode(NeutralModeValue mode) {
        wristConfig.withNeutralMode(mode); return this;
    }

    public TelescopingArmWithWristConfig withWristInverted(InvertedValue invert) {
        wristConfig.withInverted(invert); return this;
    }

    public TelescopingArmWithWristConfig withWristStatorCurrentLimit(Current limit) {
        wristConfig.withStatorCurrentLimit(limit); return this;
    }

    public TelescopingArmWithWristConfig withWristSupplyCurrentLimit(Current limit) {
        wristConfig.withSupplyCurrentLimit(limit); return this;
    }

    public TelescopingArmWithWristConfig withWristPositionTolerance(Rotation2d tolerance) {
        wristConfig.withPositionTolerance(tolerance); return this;
    }

    public TelescopingArmWithWristConfig withWristPowerPriority(PowerPriority priority) {
        wristConfig.withPowerPriority(priority); return this;
    }

    public TelescopingArmWithWristConfig withWristCANcoder(int canId) {
        wristConfig.withCANcoder(canId); return this;
    }

    public TelescopingArmWithWristConfig withWristCANcoder(int canId, String canbus) {
        wristConfig.withCANcoder(canId, canbus); return this;
    }

    public TelescopingArmWithWristConfig withWristCANcoderOffset(double offsetRotations) {
        wristConfig.withCANcoderOffset(offsetRotations); return this;
    }

    public TelescopingArmWithWristConfig withWristExtraConfigs(Consumer<TalonFXConfiguration> consumer) {
        wristConfig.withExtraConfigs(consumer); return this;
    }

    public TelescopingArmWithWristConfig withWristExtraSimConfigs(Consumer<TalonFXConfiguration> consumer) {
        wristConfig.withExtraSimConfigs(consumer); return this;
    }

    // ── Override withPivotMotor / withExtensionMotor for fluent chaining ──────

    @Override
    public TelescopingArmWithWristConfig withPivotMotor(TalonFX motor, DCMotor motorModel,
            FollowerConfig... followers) {
        super.withPivotMotor(motor, motorModel, followers);
        return this;
    }

    @Override
    public TelescopingArmWithWristConfig withExtensionMotor(TalonFX motor, DCMotor motorModel,
            FollowerConfig... followers) {
        super.withExtensionMotor(motor, motorModel, followers);
        return this;
    }
}
