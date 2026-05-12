package com.teamscreamrobotics.motorcontrol.example;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.motorcontrol.Arm;
import com.teamscreamrobotics.motorcontrol.ArmConfig;
import com.teamscreamrobotics.motorcontrol.Elevator;
import com.teamscreamrobotics.motorcontrol.ElevatorConfig;
import com.teamscreamrobotics.motorcontrol.FollowerConfig;
import com.teamscreamrobotics.motorcontrol.Flywheel;
import com.teamscreamrobotics.motorcontrol.FlywheelConfig;
import com.teamscreamrobotics.motorcontrol.Pivot;
import com.teamscreamrobotics.motorcontrol.PivotConfig;
import com.teamscreamrobotics.motorcontrol.Roller;
import com.teamscreamrobotics.motorcontrol.RollerConfig;
import com.teamscreamrobotics.motorcontrol.TalonFXWrapper;
import com.teamscreamrobotics.motorcontrol.TelescopingArm;
import com.teamscreamrobotics.motorcontrol.TelescopingArmConfig;
import com.teamscreamrobotics.motorcontrol.TelescopingArmWithWristConfig;
import com.teamscreamrobotics.motorcontrol.Turret;
import com.teamscreamrobotics.motorcontrol.TurretConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import static com.teamscreamrobotics.motorcontrol.SmartMechanism.gearing;

/**
 * Usage examples for the SCREAMLib mechanism API.
 *
 * <p>
 * All mechanisms are constructed with a config object that owns its motor(s)
 * internally.
 * Hardware is applied to the motor controller via {@code applyBuilt()} inside
 * each mechanism
 * constructor — no separate reconfigure step needed.
 */
public final class Example {

        // ── Arm ───────────────────────────────────────────────────────────────────

        static Arm buildArm(SubsystemBase subsystem) {
                return new Arm(new ArmConfig(
                                new TalonFXWrapper(new TalonFX(10), DCMotor.getKrakenX60(1), subsystem))
                                .withGearing(gearing(60.0))
                                .withNeutralMode(NeutralModeValue.Brake)
                                .withStatorCurrentLimit(Amps.of(60))
                                .withFeedforward(new ArmFeedforward(0.1, 0.4, 2.0))
                                .withSimFeedforward(new ArmFeedforward(0.0, 0.35, 1.8))
                                .withClosedLoopController(
                                                10.0, 0.0, 0.0,
                                                RotationsPerSecond.of(2.0), RotationsPerSecondPerSecond.of(4.0))
                                .withSimClosedLoopController(
                                                40.0, 0.0, 0.0,
                                                RotationsPerSecond.of(2.0), RotationsPerSecondPerSecond.of(4.0))
                                .withMotorModel(DCMotor.getKrakenX60(1))
                                .withLength(Meters.of(0.6))
                                .withMass(Kilograms.of(3.0))
                                .withHardLimit(Rotation2d.fromDegrees(-30), Rotation2d.fromDegrees(90))
                                .withStartingPosition(Rotation2d.fromDegrees(0))
                                .withHorizontalZero(Rotation2d.fromDegrees(0))
                                .withPositionTolerance(Rotation2d.fromDegrees(1.0)));
        }

        // ── Elevator ──────────────────────────────────────────────────────────────

        static Elevator buildElevator(SubsystemBase subsystem) {
                return new Elevator(new ElevatorConfig(
                                new TalonFXWrapper(new TalonFX(20), DCMotor.getKrakenX60(1), subsystem,
                                                new FollowerConfig(21)))
                                .withGearing(gearing(5.0))
                                .withMechanismCircumference(Meters.of(0.05 * 2 * Math.PI))
                                .withNeutralMode(NeutralModeValue.Brake)
                                .withStatorCurrentLimit(Amps.of(80))
                                .withFeedforward(new ElevatorFeedforward(0.1, 0.12, 0.4))
                                .withClosedLoopController(
                                                8.0, 0.0, 0.0,
                                                RotationsPerSecond.of(3.0), RotationsPerSecondPerSecond.of(6.0))
                                .withMotorModel(DCMotor.getKrakenX60(1))
                                .withMass(Kilograms.of(8.0))
                                .withHardLimits(Meters.of(0.0), Meters.of(1.2))
                                .withSoftLimits(Meters.of(0.01), Meters.of(1.18))
                                .withStartingHeight(Meters.of(0.0))
                                .withPositionTolerance(Meters.of(0.01)));
        }

        // ── Pivot (full-rotation, FusedCANcoder, with escape hatch) ──────────────

        static Pivot buildPivotWithCANcoder(SubsystemBase subsystem) {
                return new Pivot(new PivotConfig(
                                new TalonFXWrapper(new TalonFX(30), DCMotor.getKrakenX60(1), subsystem))
                                .withGearing(gearing(30.0))
                                .withNeutralMode(NeutralModeValue.Brake)
                                .withStatorCurrentLimit(Amps.of(40))
                                .withFeedbackType(FeedbackSensorSourceValue.FusedCANcoder, 31)
                                .withRotorToSensorRatio(gearing(30.0))
                                .withCANcoder(31)
                                .withCANcoderOffset(-0.125)
                                .withClosedLoopController(6.0, 0.0, 0.1)
                                .withMotorModel(DCMotor.getKrakenX60(1))
                                .withMOI(Meters.of(0.15), Kilograms.of(1.5))
                                .withWrapping(Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(180))
                                .withPositionTolerance(Rotation2d.fromDegrees(2.0))
                                // Escape hatch: set a custom CTRE config field not exposed by PivotConfig
                                .withExtraConfigs((TalonFXConfiguration cfg) -> {
                                        cfg.MotorOutput.DutyCycleNeutralDeadband = 0.04;
                                }));
        }

        // ── FlyWheel (two motors, opposite directions) ────────────────────────────

        static Flywheel buildFlyWheel(SubsystemBase subsystem) {
                return new Flywheel(new FlywheelConfig(
                                new TalonFXWrapper(new TalonFX(40), DCMotor.getKrakenX60(1), subsystem,
                                                new FollowerConfig(41, true))) // follower opposes leader
                                .withGearing(gearing(1.0))
                                .withNeutralMode(NeutralModeValue.Coast)
                                .withStatorCurrentLimit(Amps.of(60))
                                .withFeedforward(new SimpleMotorFeedforward(0.1, 0.11))
                                .withSimFeedforward(new SimpleMotorFeedforward(0.0, 0.10))
                                .withClosedLoopController(0.5, 0.0, 0.0)
                                .withMotorModel(DCMotor.getKrakenX60(1))
                                .withDiameter(Meters.of(0.1))
                                .withMass(Kilograms.of(0.5))
                                .withVelocityTolerance(RPM.of(50)));
        }

        // ── Roller ────────────────────────────────────────────────────────────────

        static Roller buildRoller(SubsystemBase subsystem) {
                return new Roller(new RollerConfig(
                                new TalonFXWrapper(new TalonFX(50), DCMotor.getKrakenX60(1), subsystem))
                                .withGearing(gearing(4.0))
                                .withNeutralMode(NeutralModeValue.Brake)
                                .withStatorCurrentLimit(Amps.of(40))
                                .withClosedLoopController(0.3, 0.0, 0.0)
                                .withFeedforward(new SimpleMotorFeedforward(0.1, 0.08))
                                .withMotorModel(DCMotor.getKrakenX60(1))
                                .withDiameter(Meters.of(0.05))
                                .withMass(Kilograms.of(0.1)));
        }

        // ── Turret ────────────────────────────────────────────────────────────────

        static Turret buildTurret(SubsystemBase subsystem) {
                return new Turret(new TurretConfig(
                                new TalonFXWrapper(new TalonFX(60), DCMotor.getKrakenX60(1), subsystem))
                                .withGearing(gearing(45.0))
                                .withNeutralMode(NeutralModeValue.Brake)
                                .withStatorCurrentLimit(Amps.of(40))
                                .withClosedLoopController(
                                                8.0, 0.0, 0.2,
                                                RotationsPerSecond.of(3.0), RotationsPerSecondPerSecond.of(6.0))
                                .withMotorModel(DCMotor.getKrakenX60(1))
                                .withMOI(Meters.of(0.2), Kilograms.of(2.0))
                                .withWrapping(Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(180))
                                .withPositionTolerance(Rotation2d.fromDegrees(1.5)));
        }

        // ── TelescopingArm ────────────────────────────────────────────────────────

        static TelescopingArm buildTelescopingArm(SubsystemBase subsystem) {
                return new TelescopingArm(new TelescopingArmConfig(subsystem)
                                .withPivotMotor(new TalonFX(70), DCMotor.getKrakenX60(1))
                                .withExtensionMotor(new TalonFX(71), DCMotor.getKrakenX60(1))
                                .withPivotGearing(gearing(40.0))
                                .withPivotMOI(Meters.of(0.2), Kilograms.of(2.5))
                                .withPivotLimits(Rotation2d.fromDegrees(-20), Rotation2d.fromDegrees(110))
                                .withStartingAngle(Rotation2d.fromDegrees(0))
                                .withPivotClosedLoopController(
                                                5.0, 0.0, 0.1,
                                                RotationsPerSecond.of(2.0), RotationsPerSecondPerSecond.of(4.0))
                                .withExtensionGearing(gearing(5.0))
                                .withExtensionMechanismCircumference(Meters.of(0.04 * 2 * Math.PI))
                                .withExtensionLimits(Meters.of(0.35), Meters.of(1.1))
                                .withStartingExtension(Meters.of(0.35))
                                .withExtensionClosedLoopController(
                                                6.0, 0.0, 0.0,
                                                RotationsPerSecond.of(4.0), RotationsPerSecondPerSecond.of(8.0))
                                .withPivotGravity(0.65)
                                .withTaskSpaceConstraints(1.5, 3.0));
        }

        // ── TelescopingArmWithWrist ───────────────────────────────────────────────

        static TelescopingArm buildTelescopingArmWithWrist(SubsystemBase subsystem) {
                var cfg = new TelescopingArmWithWristConfig(subsystem)
                                .withPivotMotor(new TalonFX(70), DCMotor.getKrakenX60(1))
                                .withExtensionMotor(new TalonFX(71), DCMotor.getKrakenX60(1))
                                .withWristMotor(new TalonFX(72), DCMotor.getKrakenX60(1));
                cfg.withPivotGearing(gearing(40.0))
                                .withStartingAngle(Rotation2d.fromDegrees(0))
                                .withPivotClosedLoopController(5.0, 0.0, 0.1)
                                .withExtensionGearing(gearing(5.0))
                                .withExtensionMechanismCircumference(Meters.of(0.04 * 2 * Math.PI))
                                .withExtensionLimits(Meters.of(0.35), Meters.of(1.1))
                                .withStartingExtension(Meters.of(0.35))
                                .withExtensionClosedLoopController(6.0, 0.0, 0.0);
                cfg.withWristGearing(gearing(20.0))
                                .withWristLimits(Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(90))
                                .withWristClosedLoopController(8.0, 0.0, 0.2);
                return new TelescopingArm(cfg);
        }
}
