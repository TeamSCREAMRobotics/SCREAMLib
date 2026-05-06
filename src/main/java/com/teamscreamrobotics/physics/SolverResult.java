package com.teamscreamrobotics.physics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

/**
 * Immutable output of one {@link ShootOnTheMoveSolver} cycle.
 *
 * @param <T> the shot-parameter type
 */
public record SolverResult<T extends ShotParameters.Interpolatable<T> & ShotParameters.HasTimeOfFlight>(
        T shotParameters,
        Rotation2d virtualHeading,
        Translation2d virtualTarget2d,
        double effectiveDistance,
        double radialVelocity,
        double tangentialVelocity,
        int iterationsUsed,
        boolean converged
) {
    /**
     * Logs all fields to AdvantageKit under the given prefix.
     * Shot parameters are not logged individually (no reflection-based field walking).
     *
     * @param prefix key prefix, e.g. {@code "Shooter/SOTM/"}
     */
    public void log(String prefix) {
        Logger.recordOutput(prefix + "VirtualHeadingDeg", virtualHeading.getDegrees());
        Logger.recordOutput(prefix + "VirtualTargetX", virtualTarget2d.getX());
        Logger.recordOutput(prefix + "VirtualTargetY", virtualTarget2d.getY());
        Logger.recordOutput(prefix + "EffectiveDistance", effectiveDistance);
        Logger.recordOutput(prefix + "RadialVelocityMPS", radialVelocity);
        Logger.recordOutput(prefix + "TangentialVelocityMPS", tangentialVelocity);
        Logger.recordOutput(prefix + "IterationsUsed", iterationsUsed);
        Logger.recordOutput(prefix + "Converged", converged);
    }
}
