package com.teamscreamrobotics.physics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Stateless shoot-on-the-move solver.
 *
 * <p>Call {@link #solve} every robot loop. The result is valid for that cycle only
 * and can be discarded; there is no internal state to manage.
 *
 * <p><b>Field-relative speeds:</b> {@code fieldRelativeSpeeds} MUST be in the field
 * frame, not the robot frame. Convert if needed:
 * <pre>
 *   ChassisSpeeds fieldSpeeds =
 *       ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());
 * </pre>
 *
 * <h2>Usage example</h2>
 * <pre>
 *   // Define season-specific shot parameters (once per season)
 *   record RebuiltShotParams(double flywheelRPM, double hoodDegrees,
 *                               double timeOfFlightSeconds)
 *       implements ShotParameters.Interpolatable&lt;RebuiltShotParams&gt;,
 *                  ShotParameters.HasTimeOfFlight {
 *     public RebuiltShotParams interpolate(RebuiltShotParams other, double t) {
 *       return new RebuiltShotParams(
 *         MathUtil.interpolate(flywheelRPM, other.flywheelRPM, t),
 *         MathUtil.interpolate(hoodDegrees, other.hoodDegrees, t),
 *         MathUtil.interpolate(timeOfFlightSeconds, other.timeOfFlightSeconds, t)
 *       );
 *     }
 *   }
 *
 *   // Build lookup table from measured data (in constants)
 *   ShotLookup&lt;RebuiltShotParams&gt; lookup = new ShotLookup&lt;&gt;()
 *       .add(1.5, new RebuiltShotParams(3200, 45.0, 0.18))
 *       .add(2.5, new RebuiltShotParams(3600, 42.0, 0.25))
 *       .add(4.0, new RebuiltShotParams(4200, 38.0, 0.36));
 *
 *   // Configure solver (once, stored in subsystem)
 *   ShootOnTheMoveSolver&lt;RebuiltShotParams&gt; solver = new ShootOnTheMoveSolver&lt;&gt;(
 *       new SolverConfig&lt;RebuiltShotParams&gt;()
 *           .withTarget(new Translation3d(0, 5.55, 2.0))
 *           .withLookup(lookup)
 *           .withReadiness(new SolverConfig.ShotReadiness(
 *               0.05,    // heading tolerance rad
 *               100.0,   // flywheel tolerance RPM
 *               2.0,     // hood tolerance deg
 *               3.5      // max robot speed m/s
 *           ))
 *   );
 *
 *   // In periodic() -- call every loop, result is instantaneous
 *   SolverResult&lt;RebuiltShotParams&gt; result = solver.solve(
 *       drivetrain.getPose(),
 *       drivetrain.getFieldRelativeSpeeds()
 *   );
 *   result.log("Shooter/SOTM/");
 *
 *   // Check readiness before firing
 *   boolean ready = evaluator.isReady(result, new ShotReadinessEvaluator.ShooterState(
 *       Math.abs(drivetrain.getHeading().minus(result.virtualHeading()).getRadians()),
 *       shooter.getVelocity().in(RPM),
 *       OptionalDouble.of(hood.getAngle().in(Degrees))
 *   ));
 * </pre>
 *
 * @param <T> the season-specific shot-parameter type
 */
public class ShootOnTheMoveSolver<T extends ShotParameters.Interpolatable<T> & ShotParameters.HasTimeOfFlight> {

    private final SolverConfig<T> config;

    public ShootOnTheMoveSolver(SolverConfig<T> config) {
        config.validate();
        this.config = config;
    }

    /**
     * Solves for the current cycle using the target from {@link SolverConfig}.
     */
    public SolverResult<T> solve(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
        return solve(robotPose, fieldRelativeSpeeds, config.targetPosition);
    }

    /**
     * Solves for the current cycle using an overridden target position.
     * Useful for dynamic targets (e.g. a moving alliance partner's robot).
     *
     * @param robotPose            current robot pose in field coordinates
     * @param fieldRelativeSpeeds  field-frame chassis speeds (NOT robot-relative)
     * @param targetPosition       3D field-space target to solve toward
     */
    public SolverResult<T> solve(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds,
                                  Translation3d targetPosition) {
        double rx = robotPose.getX();
        double ry = robotPose.getY();
        double tx = targetPosition.getX();
        double ty = targetPosition.getY();

        // Step 1: robot-to-target vector
        double dx = tx - rx;
        double dy = ty - ry;
        double realDistance2d = Math.sqrt(dx * dx + dy * dy);
        double targetAngle = Math.atan2(dy, dx);

        double vx = fieldRelativeSpeeds.vxMetersPerSecond;
        double vy = fieldRelativeSpeeds.vyMetersPerSecond;

        // Step 2: velocity decomposition
        double cosA = Math.cos(targetAngle);
        double sinA = Math.sin(targetAngle);
        double radialVelocity    =  vx * cosA + vy * sinA;
        double tangentialVelocity = -vx * sinA + vy * cosA;

        // Step 3: iterative convergence on radial component
        double effectiveDistance = realDistance2d;
        T params = lookup(effectiveDistance, radialVelocity);
        boolean converged = false;
        int iterations = 0;

        for (int i = 0; i < config.maxIterations; i++) {
            iterations = i + 1;
            double tof = params.timeOfFlightSeconds();
            double virtualDistance = Math.max(0.0, realDistance2d - radialVelocity * tof);
            if (Math.abs(virtualDistance - effectiveDistance) < config.convergenceThresholdMeters) {
                effectiveDistance = virtualDistance;
                converged = true;
                break;
            }
            effectiveDistance = virtualDistance;
            params = lookup(effectiveDistance, radialVelocity);
        }

        // Step 4: tangential correction (analytical, one step)
        double tof = params.timeOfFlightSeconds();
        double tangentialDisplacement = tangentialVelocity * tof;
        double vtx = tx - tangentialDisplacement * sinA;
        double vty = ty + tangentialDisplacement * cosA;

        Translation2d virtualTarget2d = new Translation2d(vtx, vty);
        Rotation2d virtualHeading = new Rotation2d(Math.atan2(vty - ry, vtx - rx));

        return new SolverResult<>(
                params,
                virtualHeading,
                virtualTarget2d,
                effectiveDistance,
                radialVelocity,
                tangentialVelocity,
                iterations,
                converged
        );
    }

    private T lookup(double distanceMeters, double radialVelocity) {
        if (config.physicsFunction != null) {
            return config.physicsFunction.compute(distanceMeters, radialVelocity);
        }
        return config.lookup.get(distanceMeters);
    }
}
