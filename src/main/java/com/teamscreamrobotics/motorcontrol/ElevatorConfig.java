package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;

public class ElevatorConfig {

    public final SmartMotorController motor;

    public Mass mass = null;
    public Distance hardLimitMin = null;
    public Distance hardLimitMax = null;
    public Distance startingHeight = Meters.of(0);
    public Angle angle = Degrees.of(90);
    public String logPrefix = null;

    public ElevatorConfig(SmartMotorController motor) {
        this.motor = motor;
    }

    public ElevatorConfig withMass(Mass mass) {
        this.mass = mass;
        return this;
    }

    public ElevatorConfig withHardLimits(Distance min, Distance max) {
        this.hardLimitMin = min;
        this.hardLimitMax = max;
        return this;
    }

    public ElevatorConfig withSoftLimits(Distance min, Distance max) {
        SmartMotorControllerConfig mc = motor.getConfig();
        double circumference = mc.mechanismCircumference.in(Meters);
        mc.withSoftLimit(
                Rotations.of(min.in(Meters) / circumference),
                Rotations.of(max.in(Meters) / circumference));
        motor.reconfigure();
        return this;
    }

    public ElevatorConfig withStartingHeight(Distance startingHeight) {
        this.startingHeight = startingHeight;
        return this;
    }

    public ElevatorConfig withAngle(Angle angle) {
        this.angle = angle;
        return this;
    }

    public ElevatorConfig withLogPrefix(String logPrefix) {
        this.logPrefix = logPrefix;
        return this;
    }

    public String resolveLogPrefix() {
        if (logPrefix != null) return logPrefix;
        return "Mechanisms/Elevator/" + motor.getConfig().subsystem.getClass().getSimpleName() + "/";
    }
}
