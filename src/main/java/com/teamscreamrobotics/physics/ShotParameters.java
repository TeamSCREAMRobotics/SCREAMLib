package com.teamscreamrobotics.physics;

/**
 * Marker interfaces for season-specific shot parameter records.
 *
 * <p>The user implements both interfaces on their own record each season:
 * <pre>
 *   public record MyShotParams(double flywheelRPM, double hoodDegrees, double timeOfFlightSeconds)
 *       implements ShotParameters.Interpolatable&lt;MyShotParams&gt;, ShotParameters.HasTimeOfFlight {
 *     public MyShotParams interpolate(MyShotParams other, double t) {
 *       return new MyShotParams(
 *         MathUtil.interpolate(flywheelRPM, other.flywheelRPM, t),
 *         MathUtil.interpolate(hoodDegrees, other.hoodDegrees, t),
 *         MathUtil.interpolate(timeOfFlightSeconds, other.timeOfFlightSeconds, t)
 *       );
 *     }
 *   }
 * </pre>
 */
public final class ShotParameters {

    private ShotParameters() {}

    /**
     * Allows a shot-parameter type to be linearly blended between two samples.
     *
     * @param <T> the concrete shot-parameter type
     */
    public interface Interpolatable<T> {
        /**
         * Returns a new instance blended {@code t} of the way from {@code this} to {@code other}.
         *
         * @param other the target sample
         * @param t     blend factor in [0, 1]
         */
        T interpolate(T other, double t);
    }

    /**
     * Guarantees the shot-parameter type exposes a time-of-flight value,
     * which the solver needs for its iterative convergence loop.
     */
    public interface HasTimeOfFlight {
        /** Time for the game piece to travel from the shooter to the target, in seconds. */
        double timeOfFlightSeconds();
    }
}
