package com.teamscreamrobotics.superstructure;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

/**
 * Decouples the superstructure graph from specific mechanism types.
 * The user provides one adapter per mechanism, wiring it to their mechanism object.
 *
 * <p>Setpoints are stored internally as doubles in each mechanism's natural units
 * (meters for elevators, degrees for arms/pivots, RPM for rollers). Typed
 * {@link SuperstructurePosition.Builder#set} overloads handle unit conversion via
 * {@link #toInternalValue} so callers never deal with raw doubles.
 */
public interface MechanismAdapter {

    String getName();

    /** Commands the mechanism toward {@code value} in its natural units. */
    void setSetpoint(double value);

    /** Returns the current mechanism value in its natural units. */
    double getCurrentValue();

    /** Returns true if the mechanism is within its own configured tolerance of the last setpoint. */
    boolean atSetpoint();

    /** Returns true if the current value is within {@code tolerance} of {@code value}. */
    boolean atSetpoint(double value, double tolerance);

    /**
     * Converts a WPILib {@code Measure} (or any typed setpoint object) to the internal double
     * representation used by the graph.
     *
     * <p>Each concrete adapter overrides this to accept its expected measure type and convert to
     * its natural unit. The default implementation throws {@link UnsupportedOperationException}.
     * Use {@link SuperstructurePosition.Builder#set(MechanismAdapter, double)} as an escape hatch
     * for adapters that do not implement this.
     * @param <T>
     */
    default <U extends Unit> double toInternalValue(Measure<U> measure) {
        throw new UnsupportedOperationException(
                getName() + " does not implement toInternalValue. "
                + "Use SuperstructurePosition.Builder.set(adapter, rawDouble) instead.");
    }

    /**
     * Returns the default tolerance used by the state machine's arrival check in natural units.
     * Override in each concrete adapter to match its mechanism's practical precision.
     */
    default double getDefaultTolerance() {
        return 0.5;
    }
}
