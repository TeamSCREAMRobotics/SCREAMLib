package com.teamscreamrobotics.superstructure;

import com.teamscreamrobotics.motorcontrol.Turret;

import edu.wpi.first.math.geometry.Rotation2d;

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
        turret.setAngleWithProfile(Rotation2d.fromDegrees(degrees));
    }

    @Override
    public double getCurrentValue() {
        return turret.getAngle().getDegrees();
    }

    @Override
    public boolean atSetpoint() {
        return turret.atAngle();
    }

    @Override
    public boolean atSetpoint(double degrees, double toleranceDegrees) {
        return turret.atAngle(Rotation2d.fromDegrees(degrees), Rotation2d.fromDegrees(toleranceDegrees));
    }

    /** Default tolerance: 1 degree. */
    @Override
    public double getDefaultTolerance() {
        return 1.0;
    }
}
