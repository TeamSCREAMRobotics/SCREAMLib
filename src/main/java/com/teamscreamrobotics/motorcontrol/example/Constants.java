package com.teamscreamrobotics.motorcontrol.example;

import com.ctre.phoenix6.signals.NeutralModeValue;

import com.teamscreamrobotics.motorcontrol.ArmConfig;
import com.teamscreamrobotics.motorcontrol.ElevatorConfig;
import com.teamscreamrobotics.motorcontrol.FlyWheelConfig;
import com.teamscreamrobotics.motorcontrol.PivotConfig;
import com.teamscreamrobotics.motorcontrol.SmartMotorController;
import com.teamscreamrobotics.motorcontrol.SmartMotorControllerConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import static com.teamscreamrobotics.motorcontrol.SmartMechanism.gearing;

public final class Constants {

    private Constants() {}

    // -------------------------------------------------------------------------
    // Arm -- intake/scoring pivot, Kraken X60, 60:1 reduction
    // -------------------------------------------------------------------------
    public static final class Arm {

        public static final int CAN_ID = 10;
        public static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60(1);

        public static SmartMotorControllerConfig motorConfig(SubsystemBase subsystem) {
            return new SmartMotorControllerConfig(subsystem)
                    .withMotorModel(MOTOR_MODEL)
                    .withGearing(gearing(60.0))
                    .withIdleMode(NeutralModeValue.Brake)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimit(Amps.of(80))
                    .withClosedLoopController(
                            80.0, 0.0, 2.0,
                            RotationsPerSecond.of(3.0),
                            RotationsPerSecondPerSecond.of(6.0))
                    .withFeedforward(new ArmFeedforward(0.30, 0.50, 1.20, 0.05))
                    .withOpenLoopRampRate(Seconds.of(0.1))
                    .withClosedLoopRampRate(Seconds.of(0.05));
        }

        public static ArmConfig armConfig(SmartMotorController motor) {
            return new ArmConfig(motor)
                    .withLength(Meters.of(0.50))
                    .withMass(Kilograms.of(4.0))
                    .withHardLimit(Degrees.of(-15.0), Degrees.of(105.0))
                    .withSoftLimits(Degrees.of(-10.0), Degrees.of(100.0))
                    .withStartingPosition(Degrees.of(0.0))
                    .withHorizontalZero(Degrees.of(0.0));
        }
    }

    // -------------------------------------------------------------------------
    // Elevator -- single-stage cascade, two Kraken X60, 5:1 reduction
    // 10 cm diameter drum → circumference = π × 0.10 m ≈ 0.314 m
    // -------------------------------------------------------------------------
    public static final class Elevator {

        public static final int CAN_ID = 20;
        public static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60(2);

        public static SmartMotorControllerConfig motorConfig(SubsystemBase subsystem) {
            return new SmartMotorControllerConfig(subsystem)
                    .withMotorModel(MOTOR_MODEL)
                    .withGearing(gearing(5.0))
                    .withMechanismCircumference(Meters.of(Math.PI * 0.10))
                    .withIdleMode(NeutralModeValue.Brake)
                    .withSupplyCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimit(Amps.of(120))
                    .withClosedLoopController(
                            15.0, 0.0, 0.2,
                            RotationsPerSecond.of(2.0),
                            RotationsPerSecondPerSecond.of(4.0))
                    .withFeedforward(new ElevatorFeedforward(0.10, 0.30, 0.80, 0.02))
                    .withClosedLoopRampRate(Seconds.of(0.05));
        }

        public static ElevatorConfig elevatorConfig(SmartMotorController motor) {
            return new ElevatorConfig(motor)
                    .withMass(Kilograms.of(8.0))
                    .withHardLimits(Meters.of(0.0), Meters.of(1.0))
                    .withSoftLimits(Meters.of(0.02), Meters.of(0.98))
                    .withStartingHeight(Meters.of(0.0));
        }
    }

    // -------------------------------------------------------------------------
    // Pivot (wrist) -- continuous rotation, Kraken X60, 30:1 reduction
    // -------------------------------------------------------------------------
    public static final class Pivot {

        public static final int CAN_ID = 30;
        public static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60(1);

        public static SmartMotorControllerConfig motorConfig(SubsystemBase subsystem) {
            return new SmartMotorControllerConfig(subsystem)
                    .withMotorModel(MOTOR_MODEL)
                    .withGearing(gearing(30.0))
                    .withIdleMode(NeutralModeValue.Coast)
                    .withSupplyCurrentLimit(Amps.of(30))
                    .withStatorCurrentLimit(Amps.of(60))
                    .withClosedLoopController(
                            40.0, 0.0, 0.5,
                            RotationsPerSecond.of(5.0),
                            RotationsPerSecondPerSecond.of(10.0))
                    .withClosedLoopRampRate(Seconds.of(0.02));
        }

        public static PivotConfig pivotConfig(SmartMotorController motor) {
            return new PivotConfig(motor)
                    .withHardLimit(Degrees.of(-180.0), Degrees.of(180.0))
                    .withWrapping(Degrees.of(-180.0), Degrees.of(180.0))
                    .withMOI(Meters.of(0.10), Kilograms.of(0.30))
                    .withStartingPosition(Degrees.of(0.0));
        }
    }

    // -------------------------------------------------------------------------
    // FlyWheel (shooter) -- two Kraken X60, 1:1 direct drive, 4-inch wheel
    // -------------------------------------------------------------------------
    public static final class FlyWheel {

        public static final int CAN_ID = 40;
        public static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60(2);

        public static SmartMotorControllerConfig motorConfig(SubsystemBase subsystem) {
            return new SmartMotorControllerConfig(subsystem)
                    .withMotorModel(MOTOR_MODEL)
                    .withGearing(gearing(1.0))
                    .withIdleMode(NeutralModeValue.Coast)
                    .withSupplyCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimit(Amps.of(120))
                    .withClosedLoopController(
                            0.5, 0.0, 0.0,
                            RotationsPerSecond.of(100.0),
                            RotationsPerSecondPerSecond.of(200.0))
                    .withFeedforward(new SimpleMotorFeedforward(0.10, 0.12, 0.01));
        }

        public static FlyWheelConfig flyWheelConfig(SmartMotorController motor) {
            return new FlyWheelConfig(motor)
                    .withDiameter(Meters.of(0.1016))   // 4-inch wheel
                    .withMass(Kilograms.of(0.50))
                    .withSoftLimit(RPM.of(0), RPM.of(6000));
        }
    }
}
