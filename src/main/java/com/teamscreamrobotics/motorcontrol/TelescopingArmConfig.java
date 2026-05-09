package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;

public class TelescopingArmConfig {

    public final SmartMotorController pivotMotor;
    public final SmartMotorController extensionMotor;

    public Angle startingAngle            = Degrees.of(0);
    public Angle pivotMin                 = null;
    public Angle pivotMax                 = null;
    public Distance minExtension          = Meters.of(0);
    public Distance maxExtension          = Meters.of(1.0);
    public Distance startingExtension     = Meters.of(0);
    public double pivotGravityKg          = 0.0;
    public double maxTaskVelocityMps      = 1.0;
    public double maxTaskAccelMps2        = 2.0;
    public double pivotMoiKgMetersSquared = 0.001;
    public Mass extensionMass             = null;
    public String logPrefix               = null;

    public TelescopingArmConfig(SmartMotorController pivotMotor, SmartMotorController extensionMotor) {
        this.pivotMotor = pivotMotor;
        this.extensionMotor = extensionMotor;
    }

    public TelescopingArmConfig withStartingAngle(Angle angle) {
        this.startingAngle = angle;
        return this;
    }

    public TelescopingArmConfig withPivotLimits(Angle min, Angle max) {
        this.pivotMin = min;
        this.pivotMax = max;
        return this;
    }

    public TelescopingArmConfig withExtensionLimits(Distance min, Distance max) {
        this.minExtension = min;
        this.maxExtension = max;
        return this;
    }

    public TelescopingArmConfig withStartingExtension(Distance extension) {
        this.startingExtension = extension;
        return this;
    }

    /**
     * Sets the pivot gravity feedforward constant — volts required to hold the arm
     * horizontal at full extension. Scales dynamically with extension length and angle.
     */
    public TelescopingArmConfig withPivotGravity(double kGVolts) {
        this.pivotGravityKg = kGVolts;
        return this;
    }

    public TelescopingArmConfig withTaskSpaceConstraints(double maxVelocityMps, double maxAccelMps2) {
        this.maxTaskVelocityMps = maxVelocityMps;
        this.maxTaskAccelMps2 = maxAccelMps2;
        return this;
    }

    public TelescopingArmConfig withPivotMOI(double moiKgMetersSquared) {
        this.pivotMoiKgMetersSquared = moiKgMetersSquared;
        return this;
    }

    public TelescopingArmConfig withExtensionMass(Mass mass) {
        this.extensionMass = mass;
        return this;
    }

    public TelescopingArmConfig withLogPrefix(String logPrefix) {
        this.logPrefix = logPrefix;
        return this;
    }

    public String resolveLogPrefix() {
        if (logPrefix != null) return logPrefix;
        return "Mechanisms/TelescopingArm/" + pivotMotor.getConfig().subsystem.getClass().getSimpleName() + "/";
    }
}
