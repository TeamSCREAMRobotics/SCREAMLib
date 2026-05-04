package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public class PivotConfig {

    public final SmartMotorController motor;

    public Angle hardLimitMin = null;
    public Angle hardLimitMax = null;
    public Angle startingPosition = Degrees.of(0);
    public Angle wrappingMin = null;
    public Angle wrappingMax = null;
    public double moiKgMetersSquared = 0.001;
    public String logPrefix = null;

    public PivotConfig(SmartMotorController motor) {
        this.motor = motor;
    }

    public PivotConfig withHardLimit(Angle min, Angle max) {
        this.hardLimitMin = min;
        this.hardLimitMax = max;
        return this;
    }

    public PivotConfig withStartingPosition(Angle startingPosition) {
        this.startingPosition = startingPosition;
        return this;
    }

    public PivotConfig withWrapping(Angle min, Angle max) {
        this.wrappingMin = min;
        this.wrappingMax = max;
        motor.getConfig().withContinuousWrap(true);
        motor.reconfigure();
        return this;
    }

    public PivotConfig withMOI(Distance radius, Mass mass) {
        this.moiKgMetersSquared = 0.5 * mass.in(Kilograms) * Math.pow(radius.in(Meters), 2);
        return this;
    }

    public PivotConfig withLogPrefix(String logPrefix) {
        this.logPrefix = logPrefix;
        return this;
    }

    public String resolveLogPrefix() {
        if (logPrefix != null) return logPrefix;
        return "Mechanisms/Pivot/" + motor.getConfig().subsystem.getClass().getSimpleName() + "/";
    }
}
