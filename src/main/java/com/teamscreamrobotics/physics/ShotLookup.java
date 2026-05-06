package com.teamscreamrobotics.physics;

import java.util.TreeMap;

/**
 * Distance-to-shot-parameters lookup table with linear interpolation.
 *
 * <p>Backed by a {@link TreeMap} for O(log n) neighbor lookup. Queries
 * outside the measured range are clamped to the nearest endpoint.
 *
 * <p>Usage:
 * <pre>
 *   ShotLookup&lt;MyShotParams&gt; table = new ShotLookup&lt;&gt;()
 *       .add(1.5, new MyShotParams(3200, 45.0, 0.18))
 *       .add(2.5, new MyShotParams(3600, 42.0, 0.25))
 *       .add(4.0, new MyShotParams(4200, 38.0, 0.36));
 * </pre>
 *
 * @param <T> the shot-parameter type (must implement Interpolatable and HasTimeOfFlight)
 */
public class ShotLookup<T extends ShotParameters.Interpolatable<T> & ShotParameters.HasTimeOfFlight> {

    private final TreeMap<Double, T> map = new TreeMap<>();

    /**
     * Adds a measured data point.
     *
     * @param distanceMeters horizontal distance to the target
     * @param params         measured shot parameters at that distance
     * @return this (fluent)
     */
    public ShotLookup<T> add(double distanceMeters, T params) {
        map.put(distanceMeters, params);
        return this;
    }

    /**
     * Returns interpolated shot parameters for the given distance.
     * Clamps to the nearest endpoint when outside the measured range.
     *
     * @param distanceMeters the query distance
     * @return interpolated (or boundary-clamped) shot parameters
     * @throws IllegalStateException if the table is empty
     */
    public T get(double distanceMeters) {
        if (map.isEmpty()) throw new IllegalStateException("ShotLookup table is empty");

        Double lo = map.floorKey(distanceMeters);
        Double hi = map.ceilingKey(distanceMeters);

        if (lo == null) return map.firstEntry().getValue(); // below range
        if (hi == null) return map.lastEntry().getValue();  // above range
        if (lo.equals(hi)) return map.get(lo);             // exact match

        double t = (distanceMeters - lo) / (hi - lo);
        return map.get(lo).interpolate(map.get(hi), t);
    }
}
