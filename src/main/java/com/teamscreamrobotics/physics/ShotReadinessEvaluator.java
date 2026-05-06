package com.teamscreamrobotics.physics;

import org.littletonrobotics.junction.Logger;

import java.util.OptionalDouble;

/**
 * Evaluates whether the robot is in a state safe to fire.
 *
 * @param <T> the shot-parameter type
 */
public class ShotReadinessEvaluator<T extends ShotParameters.Interpolatable<T> & ShotParameters.HasTimeOfFlight> {

    /**
     * Live shooter state for one readiness check.
     *
     * @param currentHeadingErrorRadians absolute difference between current heading and virtual heading
     * @param currentFlywheelRPM         flywheel speed as measured
     * @param currentHoodDegrees         hood angle; {@link OptionalDouble#empty()} for fixed-angle shooters
     */
    public record ShooterState(
            double currentHeadingErrorRadians,
            double currentFlywheelRPM,
            OptionalDouble currentHoodDegrees
    ) {}

    /**
     * Which readiness conditions are currently satisfied.
     */
    public record ReadinessReport(
            boolean headingReady,
            boolean flywheelReady,
            boolean hoodReady,
            boolean speedWithinLimit,
            boolean converged
    ) {
        /** Returns true if all five conditions are satisfied. */
        public boolean allReady() {
            return headingReady && flywheelReady && hoodReady && speedWithinLimit && converged;
        }

        /**
         * Logs all fields to AdvantageKit under the given prefix.
         *
         * @param prefix key prefix, e.g. {@code "Shooter/Readiness/"}
         */
        public void log(String prefix) {
            Logger.recordOutput(prefix + "HeadingReady", headingReady);
            Logger.recordOutput(prefix + "FlywheelReady", flywheelReady);
            Logger.recordOutput(prefix + "HoodReady", hoodReady);
            Logger.recordOutput(prefix + "SpeedWithinLimit", speedWithinLimit);
            Logger.recordOutput(prefix + "Converged", converged);
            Logger.recordOutput(prefix + "AllReady", allReady());
        }
    }

    private final SolverConfig<T> config;

    public ShotReadinessEvaluator(SolverConfig<T> config) {
        this.config = config;
    }

    /**
     * Returns {@code true} iff all readiness conditions are met.
     *
     * @param result       output of the most recent {@link ShootOnTheMoveSolver#solve} call
     * @param state        current shooter hardware state
     * @param robotSpeedMetersPerSec current robot speed magnitude
     */
    public boolean isReady(SolverResult<T> result, ShooterState state, double robotSpeedMetersPerSec) {
        return evaluate(result, state, robotSpeedMetersPerSec).allReady();
    }

    /**
     * Returns a full {@link ReadinessReport} describing which conditions are met.
     *
     * @param result                 output of the most recent solve
     * @param state                  current shooter hardware state
     * @param robotSpeedMetersPerSec current robot speed magnitude
     */
    public ReadinessReport evaluate(SolverResult<T> result, ShooterState state, double robotSpeedMetersPerSec) {
        SolverConfig.ShotReadiness r = config.readiness;

        boolean headingReady  = state.currentHeadingErrorRadians() < r.headingToleranceRadians();
        boolean flywheelReady = Math.abs(state.currentFlywheelRPM()
                - getTargetRPM(result, state)) < r.flywheelToleranceRPM();
        boolean hoodReady     = evaluateHood(result, state, r.hoodToleranceDegrees());
        boolean speedOk       = robotSpeedMetersPerSec < r.maxRobotSpeedMetersPerSec();
        boolean converged     = result.converged();

        return new ReadinessReport(headingReady, flywheelReady, hoodReady, speedOk, converged);
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    private double getTargetRPM(SolverResult<T> result, ShooterState state) {
        try {
            return (double) result.shotParameters().getClass()
                    .getMethod("flywheelRPM").invoke(result.shotParameters());
        } catch (Exception e) {
            // T has no flywheelRPM() — treat check as trivially passing
            return state.currentFlywheelRPM();
        }
    }

    private boolean evaluateHood(SolverResult<T> result, ShooterState state, double toleranceDeg) {
        if (state.currentHoodDegrees().isEmpty()) return true;
        double targetHood;
        try {
            targetHood = (double) result.shotParameters().getClass()
                    .getMethod("hoodDegrees").invoke(result.shotParameters());
        } catch (Exception e) {
            return true; // T has no hoodDegrees — not applicable
        }
        return Math.abs(state.currentHoodDegrees().getAsDouble() - targetHood) < toleranceDeg;
    }
}
