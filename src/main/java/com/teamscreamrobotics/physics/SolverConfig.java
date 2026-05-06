package com.teamscreamrobotics.physics;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Configuration for {@link ShootOnTheMoveSolver}.
 *
 * <p>Exactly one of {@link #lookup} or {@link #physicsFunction} must be set before
 * passing this config to the solver.
 *
 * @param <T> the shot-parameter type
 */
public class SolverConfig<T extends ShotParameters.Interpolatable<T> & ShotParameters.HasTimeOfFlight> {

    /**
     * A function that computes shot parameters analytically from distance and radial velocity.
     * Use this instead of a {@link ShotLookup} when you have a physics model.
     *
     * @param <T> the shot-parameter type
     */
    @FunctionalInterface
    public interface ShotPhysicsFunction<T> {
        /**
         * @param distanceMeters            effective distance to the target (radial-adjusted)
         * @param radialVelocityMetersPerSec robot's radial velocity (positive = moving toward target)
         * @return computed shot parameters
         */
        T compute(double distanceMeters, double radialVelocityMetersPerSec);
    }

    /**
     * Thresholds used by {@link ShotReadinessEvaluator} to determine if the robot is safe to fire.
     */
    public record ShotReadiness(
            double headingToleranceRadians,
            double flywheelToleranceRPM,
            double hoodToleranceDegrees,
            double maxRobotSpeedMetersPerSec
    ) {
        public ShotReadiness() {
            this(0.05, 100.0, 2.0, 4.5);
        }
    }

    // ── Required ──────────────────────────────────────────────────────────────
    Translation3d targetPosition = null;

    // ── Data source (exactly one must be set) ─────────────────────────────────
    ShotLookup<T> lookup = null;
    ShotPhysicsFunction<T> physicsFunction = null;

    // ── Convergence ───────────────────────────────────────────────────────────
    int maxIterations = 7;
    double convergenceThresholdMeters = 0.01;

    // ── Readiness ─────────────────────────────────────────────────────────────
    ShotReadiness readiness = new ShotReadiness();

    public SolverConfig<T> withTarget(Translation3d target) {
        this.targetPosition = target;
        return this;
    }

    public SolverConfig<T> withLookup(ShotLookup<T> lookup) {
        this.lookup = lookup;
        return this;
    }

    public SolverConfig<T> withPhysicsFunction(ShotPhysicsFunction<T> fn) {
        this.physicsFunction = fn;
        return this;
    }

    public SolverConfig<T> withMaxIterations(int maxIterations) {
        this.maxIterations = maxIterations;
        return this;
    }

    public SolverConfig<T> withConvergenceThreshold(double meters) {
        this.convergenceThresholdMeters = meters;
        return this;
    }

    public SolverConfig<T> withReadiness(ShotReadiness readiness) {
        this.readiness = readiness;
        return this;
    }

    void validate() {
        if (targetPosition == null)
            throw new IllegalStateException("SolverConfig: targetPosition must be set");
        if (lookup == null && physicsFunction == null)
            throw new IllegalStateException("SolverConfig: exactly one of lookup or physicsFunction must be set");
        if (lookup != null && physicsFunction != null)
            throw new IllegalStateException("SolverConfig: lookup and physicsFunction are mutually exclusive");
    }
}
