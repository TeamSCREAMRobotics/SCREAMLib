package com.teamscreamrobotics.superstructure;

import com.teamscreamrobotics.motorcontrol.Arm;

import edu.wpi.first.math.geometry.Rotation2d;

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
        arm.setAngleWithProfile(Rotation2d.fromDegrees(degrees));
    }

    @Override
    public double getCurrentValue() {
        return arm.getAngle().getDegrees();
    }

    @Override
    public boolean atSetpoint() {
        return arm.atAngle();
    }

    @Override
    public boolean atSetpoint(double degrees, double toleranceDegrees) {
        return arm.atAngle(Rotation2d.fromDegrees(degrees), Rotation2d.fromDegrees(toleranceDegrees));
    }

    /** Default tolerance: 1 degree. */
    @Override
    public double getDefaultTolerance() {
        return 1.0;
    }
}
