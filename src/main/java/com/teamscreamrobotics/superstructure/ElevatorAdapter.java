package com.teamscreamrobotics.superstructure;

import com.teamscreamrobotics.motorcontrol.Elevator;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Meters;

/** Adapts {@link Elevator} to the superstructure graph. Setpoints are in meters. */
public class ElevatorAdapter implements MechanismAdapter {

    private final String name;
    private final Elevator elevator;

    public ElevatorAdapter(Elevator elevator) {
        this(elevator.getClass().getSimpleName(), elevator);
    }

    public ElevatorAdapter(String name, Elevator elevator) {
        this.name = name;
        this.elevator = elevator;
    }

    @Override public String getName() { return name; }

    @Override
    public void setSetpoint(double meters) {
        elevator.setHeight(Meters.of(meters));
    }

    @Override
    public double getCurrentValue() {
        return elevator.getHeight().in(Meters);
    }

    @Override
    public boolean atSetpoint() {
        return elevator.atHeight();
    }

    @Override
    public boolean atSetpoint(double meters, double toleranceMeters) {
        return elevator.atHeight(Meters.of(meters), Meters.of(toleranceMeters));
    }

    /** Accepts {@link Distance}; returns meters as a double. */
    @Override
    public <U extends Unit> double toInternalValue(Measure<U> measure) {
        if (measure instanceof Distance distance) {
            return distance.in(Meters);
        }
        throw new IllegalArgumentException(
                "ElevatorAdapter '" + name + "' expects Measure<Distance>, got: " + measure.getClass().getSimpleName());
    }

    /** Default tolerance: 1 cm. */
    @Override
    public double getDefaultTolerance() {
        return 0.01;
    }
}
