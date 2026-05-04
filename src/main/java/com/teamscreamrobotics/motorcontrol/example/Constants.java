package com.teamscreamrobotics.motorcontrol.example;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.motorcontrol.ArmConfig;
import com.teamscreamrobotics.motorcontrol.ElevatorConfig;
import com.teamscreamrobotics.motorcontrol.FlyWheelConfig;
import com.teamscreamrobotics.motorcontrol.PivotConfig;
import com.teamscreamrobotics.motorcontrol.SmartMotorControllerConfig;
import com.teamscreamrobotics.motorcontrol.SmartMotorControllerConfig.CANcoderConfig;
import com.teamscreamrobotics.motorcontrol.SmartMotorControllerConfig.FollowerConfig;
import com.teamscreamrobotics.motorcontrol.TalonFXWrapper;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import static com.teamscreamrobotics.motorcontrol.SmartMechanism.gearing;

public final class Constants {

    public static final class Arm {
        public static final int CAN_ID = 10;
        public static final int CANCODER_ID = 11;

        public static TalonFXWrapper motor(SubsystemBase subsystem) {
            SmartMotorControllerConfig cfg = new SmartMotorControllerConfig(subsystem)
                    .withGearing(gearing(60.0))
                    .withIdleMode(NeutralModeValue.Brake)
                    .withStatorCurrentLimit(Amps.of(60))
                    // WPILib ArmFeedforward — kG compensates gravity; kV/kA tune following
                    .withFeedforward(new ArmFeedforward(0.1, 0.4, 2.0))
                    .withClosedLoopController(
                            10.0, 0.0, 0.0,
                            RotationsPerSecond.of(2.0),
                            RotationsPerSecondPerSecond.of(4.0))
                    .withCANcoder(new CANcoderConfig(CANCODER_ID, "", -0.25, true))
                    .withMotorModel(DCMotor.getKrakenX60(1))
                    .withPositionTolerance(Degrees.of(1.0));
            return new TalonFXWrapper(new TalonFX(CAN_ID), DCMotor.getKrakenX60(1), cfg);
        }

        public static ArmConfig armConfig(TalonFXWrapper motor) {
            return new ArmConfig(motor)
                    .withLength(Meters.of(0.6))
                    .withMass(Kilograms.of(3.0))
                    .withHardLimit(Degrees.of(-30), Degrees.of(90))
                    .withStartingPosition(Degrees.of(0))
                    // encoder zero is at horizontal, so no offset needed
                    .withHorizontalZero(Degrees.of(0));
        }
    }

    public static final class Elevator {
        public static final int CAN_ID = 20;
        // Single-motor model: Phoenix propagates sim state to the follower automatically.
        // Using getKrakenX60(2) would double the plant's stall torque, overpowering the sim.
        public static final int FOLLOWER_CAN_ID = 21;

        public static TalonFXWrapper motor(SubsystemBase subsystem) {
            SmartMotorControllerConfig cfg = new SmartMotorControllerConfig(subsystem)
                    .withGearing(gearing(5.0))
                    .withMechanismCircumference(Meters.of(0.05 * 2 * Math.PI)) // 5 cm drum radius
                    .withIdleMode(NeutralModeValue.Brake)
                    .withStatorCurrentLimit(Amps.of(80))
                    // CTRE onboard feedforward with elevator gravity compensation
                    .withClosedLoopController(
                            8.0, 0.0, 0.0,
                            0.1, 0.12, 0.01, 0.4,
                            GravityTypeValue.Elevator_Static,
                            RotationsPerSecond.of(3.0),
                            RotationsPerSecondPerSecond.of(6.0))
                    .withMotorModel(DCMotor.getKrakenX60(1))
                    .withFollowers(new FollowerConfig(FOLLOWER_CAN_ID))
                    .withPositionTolerance(Rotations.of(0.02));
            return new TalonFXWrapper(new TalonFX(CAN_ID), DCMotor.getKrakenX60(1), cfg);
        }

        public static ElevatorConfig elevatorConfig(TalonFXWrapper motor) {
            return new ElevatorConfig(motor)
                    .withMass(Kilograms.of(8.0))
                    .withHardLimits(Meters.of(0.0), Meters.of(1.2))
                    .withStartingHeight(Meters.of(0.0));
        }
    }

    public static final class Pivot {
        public static final int CAN_ID = 30;

        public static TalonFXWrapper motor(SubsystemBase subsystem) {
            SmartMotorControllerConfig cfg = new SmartMotorControllerConfig(subsystem)
                    .withGearing(gearing(30.0))
                    .withIdleMode(NeutralModeValue.Brake)
                    .withStatorCurrentLimit(Amps.of(40))
                    // Plain PID — no Motion Magic, no gravity compensation needed
                    .withClosedLoopController(6.0, 0.0, 0.1)
                    .withMotorModel(DCMotor.getKrakenX60(1))
                    .withPositionTolerance(Degrees.of(2.0));
            return new TalonFXWrapper(new TalonFX(CAN_ID), DCMotor.getKrakenX60(1), cfg);
        }

        public static PivotConfig pivotConfig(TalonFXWrapper motor) {
            return new PivotConfig(motor)
                    .withMOI(Meters.of(0.15), Kilograms.of(1.5))
                    .withWrapping(Degrees.of(-180), Degrees.of(180));
        }
    }

    public static final class FlyWheel {
        public static final int CAN_ID = 40;
        // Single-motor model: see Elevator note above.
        public static final int FOLLOWER_CAN_ID = 41;

        public static TalonFXWrapper motor(SubsystemBase subsystem) {
            SmartMotorControllerConfig cfg = new SmartMotorControllerConfig(subsystem)
                    .withGearing(gearing(1.0))
                    .withIdleMode(NeutralModeValue.Coast)
                    .withStatorCurrentLimit(Amps.of(60))
                    // Plain PID for velocity control — no Motion Magic needed for flywheels
                    .withClosedLoopController(0.5, 0.0, 0.0)
                    .withMotorModel(DCMotor.getKrakenX60(1))
                    .withFollowers(new FollowerConfig(FOLLOWER_CAN_ID))
                    .withVelocityTolerance(RPM.of(50));
            return new TalonFXWrapper(new TalonFX(CAN_ID), DCMotor.getKrakenX60(1), cfg);
        }

        public static FlyWheelConfig flyWheelConfig(TalonFXWrapper motor) {
            return new FlyWheelConfig(motor)
                    .withDiameter(Meters.of(0.1))
                    .withMass(Kilograms.of(0.5));
        }
    }
}
