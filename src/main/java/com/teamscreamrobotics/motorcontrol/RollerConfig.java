package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class RollerConfig {

    public final SmartMotorController motor;

    public Distance diameter = null;
    public AngularVelocity upperSoftLimit = null;
    public AngularVelocity lowerSoftLimit = null;
    public String logPrefix = null;

    public RollerConfig(SmartMotorController motor) {
        this.motor = motor;
    }

    public RollerConfig withDiameter(Distance diameter) {
        this.diameter = diameter;
        return this;
    }

    public RollerConfig withUpperSoftLimit(AngularVelocity limit) {
        this.upperSoftLimit = limit;
        return this;
    }

    public RollerConfig withLowerSoftLimit(AngularVelocity limit) {
        this.lowerSoftLimit = limit;
        return this;
    }

    public RollerConfig withSoftLimit(AngularVelocity lower, AngularVelocity upper) {
        this.lowerSoftLimit = lower;
        this.upperSoftLimit = upper;
        return this;
    }

    public RollerConfig withLogPrefix(String logPrefix) {
        this.logPrefix = logPrefix;
        return this;
    }

    public String resolveLogPrefix() {
        if (logPrefix != null) return logPrefix;
        return "Mechanisms/Roller/" + motor.getConfig().subsystem.getClass().getSimpleName() + "/";
    }
}
