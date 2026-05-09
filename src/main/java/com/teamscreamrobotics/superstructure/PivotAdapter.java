package com.teamscreamrobotics.superstructure;

import com.teamscreamrobotics.motorcontrol.Pivot;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

/** Adapts {@link Pivot} to the superstructure graph. Setpoints are in degrees. */
public class PivotAdapter implements MechanismAdapter {

    private final String name;
    private final Pivot pivot;

    public PivotAdapter(Pivot pivot) {
        this(pivot.getClass().getSimpleName(), pivot);
    }

    public PivotAdapter(String name, Pivot pivot) {
        this.name = name;
        this.pivot = pivot;
    }

    @Override public String getName() { return name; }

    @Override
    public void setSetpoint(double degrees) {
        pivot.setAngleWithProfile(Degrees.of(degrees));
    }

    @Override
    public double getCurrentValue() {
        return pivot.getAngle().in(Degrees);
    }

    @Override
    public boolean atSetpoint() {
        return pivot.atAngle();
    }

    @Override
    public boolean atSetpoint(double degrees, double toleranceDegrees) {
        return pivot.atAngle(Degrees.of(degrees), Degrees.of(toleranceDegrees));
    }

    /** Accepts {@link Angle}; returns degrees as a double. */
    @Override
    public <U extends Unit> double toInternalValue(Measure<U> measure) {
        if (measure instanceof Angle angle) {
            return angle.in(Degrees);
        }
        throw new IllegalArgumentException(
                "PivotAdapter '" + name + "' expects Measure<Angle>, got: " + measure.getClass().getSimpleName());
    }

    /** Default tolerance: 1 degree. */
    @Override
    public double getDefaultTolerance() {
        return 1.0;
    }
}
