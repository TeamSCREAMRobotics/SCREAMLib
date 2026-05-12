package com.teamscreamrobotics.superstructure;

import com.teamscreamrobotics.motorcontrol.Pivot;

import edu.wpi.first.math.geometry.Rotation2d;

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
        pivot.setAngleWithProfile(Rotation2d.fromDegrees(degrees));
    }

    @Override
    public double getCurrentValue() {
        return pivot.getAngle().getDegrees();
    }

    @Override
    public boolean atSetpoint() {
        return pivot.atAngle();
    }

    @Override
    public boolean atSetpoint(double degrees, double toleranceDegrees) {
        return pivot.atAngle(Rotation2d.fromDegrees(degrees), Rotation2d.fromDegrees(toleranceDegrees));
    }

    /** Default tolerance: 1 degree. */
    @Override
    public double getDefaultTolerance() {
        return 1.0;
    }
}
