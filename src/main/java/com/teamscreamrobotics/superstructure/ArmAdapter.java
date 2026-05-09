package com.teamscreamrobotics.superstructure;

import com.teamscreamrobotics.motorcontrol.Arm;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

/** Adapts {@link Arm} to the superstructure graph. Setpoints are in degrees. */
public class ArmAdapter implements MechanismAdapter {

    private final String name;
    private final Arm arm;

    public ArmAdapter(Arm arm) {
        this(arm.getClass().getSimpleName(), arm);
    }

    public ArmAdapter(String name, Arm arm) {
        this.name = name;
        this.arm = arm;
    }

    @Override public String getName() { return name; }

    @Override
    public void setSetpoint(double degrees) {
        arm.setAngleWithProfile(Degrees.of(degrees));
    }

    @Override
    public double getCurrentValue() {
        return arm.getAngle().in(Degrees);
    }

    @Override
    public boolean atSetpoint() {
        return arm.atAngle();
    }

    @Override
    public boolean atSetpoint(double degrees, double toleranceDegrees) {
        return arm.atAngle(Degrees.of(degrees), Degrees.of(toleranceDegrees));
    }

    /** Accepts {@link Angle}; returns degrees as a double. */
    @Override
    public <U extends Unit> double toInternalValue(Measure<U> measure) {
        if (measure instanceof Angle angle) {
            return angle.in(Degrees);
        }
        throw new IllegalArgumentException(
                "ArmAdapter '" + name + "' expects Measure<Angle>, got: " + measure.getClass().getSimpleName());
    }

    /** Default tolerance: 1 degree. */
    @Override
    public double getDefaultTolerance() {
        return 1.0;
    }
}
