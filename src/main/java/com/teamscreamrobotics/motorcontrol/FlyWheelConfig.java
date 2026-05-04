package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;

public class FlyWheelConfig {

    public final SmartMotorController motor;

    public Distance diameter = null;
    public Mass mass = null;
    public AngularVelocity upperSoftLimit = null;
    public AngularVelocity lowerSoftLimit = null;
    public String logPrefix = null;

    public FlyWheelConfig(SmartMotorController motor) {
        this.motor = motor;
    }

    public FlyWheelConfig withDiameter(Distance diameter) {
        this.diameter = diameter;
        return this;
    }

    public FlyWheelConfig withMass(Mass mass) {
        this.mass = mass;
        return this;
    }

    public FlyWheelConfig withUpperSoftLimit(AngularVelocity limit) {
        this.upperSoftLimit = limit;
        return this;
    }

    public FlyWheelConfig withLowerSoftLimit(AngularVelocity limit) {
        this.lowerSoftLimit = limit;
        return this;
    }

    public FlyWheelConfig withSoftLimit(AngularVelocity lower, AngularVelocity upper) {
        this.lowerSoftLimit = lower;
        this.upperSoftLimit = upper;
        return this;
    }

    public FlyWheelConfig withLogPrefix(String logPrefix) {
        this.logPrefix = logPrefix;
        return this;
    }

    public double getMOI() {
        double radius = diameter.in(Meters) / 2.0;
        return 0.5 * mass.in(Kilograms) * radius * radius;
    }

    public String resolveLogPrefix() {
        if (logPrefix != null) return logPrefix;
        return "Mechanisms/FlyWheel/" + motor.getConfig().subsystem.getClass().getSimpleName() + "/";
    }
}
