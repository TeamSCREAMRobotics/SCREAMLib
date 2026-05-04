package com.teamscreamrobotics.motorcontrol;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public interface SmartMotorController {

    @AutoLog
    public static class SmartMotorControllerInputs {
        // Master motor
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean connected = false;

        // Per-follower arrays -- index matches followers[] order in SmartMotorControllerConfig.
        // Logged as inputs (replayable) to support current-based game piece detection logic
        // that reads follower currents independently during replay.
        public double[] followerSupplyCurrentAmps = new double[0];
        public double[] followerStatorCurrentAmps = new double[0];
        public double[] followerTempCelsius = new double[0];
        public boolean[] followerConnected = new boolean[0];

        // CANcoder absolute position (0.0 / false when no CANcoder is configured)
        public double cancoderPositionRad = 0.0;
        public boolean cancoderConnected = false;
    }

    void setPosition(Angle position);
    void setVelocity(AngularVelocity velocity);
    void setVoltage(Voltage voltage);
    void setLinearPosition(Distance position);
    void setLinearVelocity(LinearVelocity velocity);
    void resetEncoder(Angle position);
    void stop();

    Angle getMechanismPosition();
    AngularVelocity getMechanismVelocity();

    default Distance getLinearPosition() {
        double circumference = getConfig().mechanismCircumference.in(Meters);
        return Meters.of(getMechanismPosition().in(Radians) / (2.0 * Math.PI) * circumference);
    }

    default LinearVelocity getLinearVelocity() {
        double circumference = getConfig().mechanismCircumference.in(Meters);
        return MetersPerSecond.of(getMechanismVelocity().in(RadiansPerSecond) / (2.0 * Math.PI) * circumference);
    }

    Voltage getVoltage();
    Current getSupplyCurrent();
    Current getStatorCurrent();
    Temperature getTemperature();

    SmartMotorControllerConfig getConfig();
    SmartMotorControllerInputsAutoLogged getInputs();
    String getLogPrefix();

    void updateInputs(SmartMotorControllerInputs inputs);
    void simIterate(double dtSeconds);
    void simUpdate(Angle position, AngularVelocity velocity);
    void reconfigure();

    default double getSimVoltage() { return 0.0; }
    default double getHorizontalZeroRad() { return 0.0; }
    default void setHorizontalZeroRad(double rad) {}

    default Angle getCANcoderPosition() {
        return Radians.of(getInputs().cancoderPositionRad);
    }
}
