package com.teamscreamrobotics.superstructure;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * A named configuration of all mechanisms in the superstructure.
 *
 * <p>Setpoints are stored internally as doubles in each mechanism's natural units.
 * Use the typed builder overloads — the compiler enforces unit correctness:
 *
 * <pre>
 *   SuperstructurePosition.builder()
 *       .set(elevatorAdapter, Meters.of(0.8))
 *       .set(armAdapter,      Degrees.of(85.0))
 *       .set(wristAdapter,    Degrees.of(45.0))
 *       .build()
 * </pre>
 *
 * Adapters are the map keys; the same object references passed to
 * {@link SuperstructureGraph#registerMechanism} must be used here.
 */
public record SuperstructurePosition(Map<MechanismAdapter, Double> mechanismSetpoints) {

    public SuperstructurePosition {
        mechanismSetpoints = Collections.unmodifiableMap(new LinkedHashMap<>(mechanismSetpoints));
    }

    /**
     * Returns the setpoint for {@code adapter} in its natural units, or {@code 0.0} if this
     * position does not specify a setpoint for that mechanism.
     */
    public double getSetpoint(MechanismAdapter adapter) {
        return mechanismSetpoints.getOrDefault(adapter, 0.0);
    }

    public static Builder builder() {
        return new Builder();
    }

    public static final class Builder {
        private final Map<MechanismAdapter, Double> setpoints = new LinkedHashMap<>();

        private Builder() {}

        /** Sets an arm setpoint. Value must be an {@link Angle} (e.g. {@code Degrees.of(-10)}). */
        public Builder set(ArmAdapter adapter, Angle setpoint) {
            setpoints.put(adapter, adapter.toInternalValue(setpoint));
            return this;
        }

        /** Sets an elevator setpoint. Value must be a {@link Distance} (e.g. {@code Meters.of(0.5)}). */
        public Builder set(ElevatorAdapter adapter, Distance setpoint) {
            setpoints.put(adapter, adapter.toInternalValue(setpoint));
            return this;
        }

        /** Sets a pivot setpoint. Value must be an {@link Angle} (e.g. {@code Degrees.of(90)}). */
        public Builder set(PivotAdapter adapter, Angle setpoint) {
            setpoints.put(adapter, adapter.toInternalValue(setpoint));
            return this;
        }

        /** Sets a turret setpoint. Value must be an {@link Angle} (e.g. {@code Degrees.of(90)}). */
        public Builder set(TurretAdapter adapter, Angle setpoint) {
            setpoints.put(adapter, adapter.toInternalValue(setpoint));
            return this;
        }

        /**
         * Generic escape hatch for custom adapters.
         * The caller is responsible for providing {@code rawValue} in the adapter's natural units.
         */
        public <T extends MechanismAdapter> Builder set(T adapter, double rawValue) {
            setpoints.put(adapter, rawValue);
            return this;
        }

        public SuperstructurePosition build() {
            return new SuperstructurePosition(setpoints);
        }
    }
}
