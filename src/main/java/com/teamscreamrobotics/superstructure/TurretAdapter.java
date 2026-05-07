package com.teamscreamrobotics.superstructure;

import com.teamscreamrobotics.motorcontrol.Turret;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

/** Adapts {@link Turret} to the superstructure graph. Setpoints are in degrees. */
public class TurretAdapter implements MechanismAdapter {

    private final String name;
    private final Turret turret;

    public TurretAdapter(Turret turret) {
        this(turret.getClass().getSimpleName(), turret);
    }

    public TurretAdapter(String name, Turret turret) {
        this.name = name;
        this.turret = turret;
    }

    @Override public String getName() { return name; }

    @Override
    public void setSetpoint(double degrees) {
        turret.setAngle(Degrees.of(degrees));
    }

    @Override
    public double getCurrentValue() {
        return turret.getAngle().in(Degrees);
    }

    @Override
    public boolean atSetpoint() {
        return turret.atAngle();
    }

    @Override
    public boolean atSetpoint(double degrees, double toleranceDegrees) {
        return turret.atAngle(Degrees.of(degrees), Degrees.of(toleranceDegrees));
    }

    /** Accepts {@link Angle}; returns degrees as a double. */
    @Override
    public <U extends Unit> double toInternalValue(Measure<U> measure) {
        if (measure instanceof Angle angle) {
            return angle.in(Degrees);
        }
        throw new IllegalArgumentException(
                "TurretAdapter '" + name + "' expects Measure<Angle>, got: " + measure.getClass().getSimpleName());
    }

    /** Default tolerance: 1 degree. */
    @Override
    public double getDefaultTolerance() {
        return 1.0;
    }
}
