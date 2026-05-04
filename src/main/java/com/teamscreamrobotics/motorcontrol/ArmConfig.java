package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;

public class ArmConfig {

    public final SmartMotorController motor;

    public Distance length = null;
    public Mass mass = null;
    public Angle hardLimitMin = null;
    public Angle hardLimitMax = null;
    public Angle startingPosition = Degrees.of(0);
    public Angle horizontalZero = Degrees.of(0);
    public String logPrefix = null;

    public ArmConfig(SmartMotorController motor) {
        this.motor = motor;
    }

    public ArmConfig withLength(Distance length) {
        this.length = length;
        return this;
    }

    public ArmConfig withMass(Mass mass) {
        this.mass = mass;
        return this;
    }

    public ArmConfig withHardLimit(Angle min, Angle max) {
        this.hardLimitMin = min;
        this.hardLimitMax = max;
        return this;
    }

    public ArmConfig withSoftLimits(Angle min, Angle max) {
        motor.getConfig().withSoftLimit(min, max);
        motor.reconfigure();
        return this;
    }

    public ArmConfig withStartingPosition(Angle startingPosition) {
        this.startingPosition = startingPosition;
        return this;
    }

    public ArmConfig withHorizontalZero(Angle horizontalZero) {
        this.horizontalZero = horizontalZero;
        return this;
    }

    public ArmConfig withLogPrefix(String logPrefix) {
        this.logPrefix = logPrefix;
        return this;
    }

    public String resolveLogPrefix() {
        if (logPrefix != null) return logPrefix;
        return "Mechanisms/Arm/" + motor.getConfig().subsystem.getClass().getSimpleName() + "/";
    }
}
