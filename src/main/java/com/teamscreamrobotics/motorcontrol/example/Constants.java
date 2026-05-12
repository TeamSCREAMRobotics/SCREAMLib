package com.teamscreamrobotics.motorcontrol.example;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.motorcontrol.ArmConfig;
import com.teamscreamrobotics.motorcontrol.ElevatorConfig;
import com.teamscreamrobotics.motorcontrol.FollowerConfig;
import com.teamscreamrobotics.motorcontrol.FlywheelConfig;
import com.teamscreamrobotics.motorcontrol.PivotConfig;
import com.teamscreamrobotics.motorcontrol.TalonFXWrapper;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import static com.teamscreamrobotics.motorcontrol.SmartMechanism.gearing;

public final class Constants {

    public static final class Arm {
        public static final int CAN_ID = 10;
        public static final int CANCODER_ID = 11;

        public static ArmConfig config(SubsystemBase subsystem) {
            return new ArmConfig(new TalonFXWrapper(new TalonFX(CAN_ID), DCMotor.getKrakenX60(1), subsystem))
                    .withGearing(gearing(60.0))
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withStatorCurrentLimit(Amps.of(60))
                    .withFeedforward(new ArmFeedforward(0.1, 0.4, 2.0))
                    .withClosedLoopController(
                            10.0, 0.0, 0.0,
                            RotationsPerSecond.of(2.0),
                            RotationsPerSecondPerSecond.of(4.0))
                    .withCANcoder(CANCODER_ID)
                    .withCANcoderOffset(-0.25)
                    .withMotorModel(DCMotor.getKrakenX60(1))
                    .withLength(Meters.of(0.6))
                    .withMass(Kilograms.of(3.0))
                    .withHardLimit(Rotation2d.fromDegrees(-30), Rotation2d.fromDegrees(90))
                    .withStartingPosition(Rotation2d.fromDegrees(0))
                    .withHorizontalZero(Rotation2d.fromDegrees(0))
                    .withPositionTolerance(Rotation2d.fromDegrees(1.0));
        }
    }

    public static final class Elevator {
        public static final int CAN_ID = 20;
        public static final int FOLLOWER_CAN_ID = 21;

        public static ElevatorConfig config(SubsystemBase subsystem) {
            return new ElevatorConfig(
                    new TalonFXWrapper(new TalonFX(CAN_ID), DCMotor.getKrakenX60(1), subsystem,
                            new FollowerConfig(FOLLOWER_CAN_ID)))
                    .withGearing(gearing(5.0))
                    .withMechanismCircumference(Meters.of(0.05 * 2 * Math.PI))
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withStatorCurrentLimit(Amps.of(80))
                    .withFeedforward(new ElevatorFeedforward(0.1, 0.12, 0.4))
                    .withClosedLoopController(
                            8.0, 0.0, 0.0,
                            RotationsPerSecond.of(3.0),
                            RotationsPerSecondPerSecond.of(6.0))
                    .withMotorModel(DCMotor.getKrakenX60(1))
                    .withMass(Kilograms.of(8.0))
                    .withHardLimits(Meters.of(0.0), Meters.of(1.2))
                    .withStartingHeight(Meters.of(0.0))
                    .withPositionTolerance(Meters.of(0.01));
        }
    }

    public static final class Pivot {
        public static final int CAN_ID = 30;

        public static PivotConfig config(SubsystemBase subsystem) {
            return new PivotConfig(new TalonFXWrapper(new TalonFX(CAN_ID), DCMotor.getKrakenX60(1), subsystem))
                    .withGearing(gearing(30.0))
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withClosedLoopController(6.0, 0.0, 0.1)
                    .withMotorModel(DCMotor.getKrakenX60(1))
                    .withMOI(Meters.of(0.15), Kilograms.of(1.5))
                    .withWrapping(Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(180))
                    .withPositionTolerance(Rotation2d.fromDegrees(2.0));
        }
    }

    public static final class FlyWheel {
        public static final int CAN_ID = 40;
        public static final int FOLLOWER_CAN_ID = 41;

        public static FlywheelConfig config(SubsystemBase subsystem) {
            return new FlywheelConfig(
                    new TalonFXWrapper(new TalonFX(CAN_ID), DCMotor.getKrakenX60(1), subsystem,
                            new FollowerConfig(FOLLOWER_CAN_ID)))
                    .withGearing(gearing(1.0))
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withStatorCurrentLimit(Amps.of(60))
                    .withClosedLoopController(0.5, 0.0, 0.0)
                    .withMotorModel(DCMotor.getKrakenX60(1))
                    .withDiameter(Meters.of(0.1))
                    .withMass(Kilograms.of(0.5))
                    .withVelocityTolerance(RPM.of(50));
        }
    }
}
