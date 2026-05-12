package com.teamscreamrobotics.vision;

import com.teamscreamrobotics.zones.RectangularPoseArea;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.function.Supplier;

/**
 * Configuration for a multi-camera {@link LimelightVision} system.
 *
 * <h2>Standard deviation formula</h2>
 * <pre>
 *   xyStdDev    = xyStdDevCoefficient    * avgTagDistance² / tagCount
 *   thetaStdDev = thetaStdDevCoefficient * avgTagDistance² / tagCount
 * </pre>
 *
 * <p><b>xyStdDevCoefficient</b>: lower = trust vision more at all distances.
 * Start around 0.005 for a well-mounted camera; increase if pose jumps.
 *
 * <p><b>thetaStdDevCoefficient</b>: almost always leave at the default (9999999) to defer
 * heading entirely to the gyro. Only lower this if you are NOT using MegaTag2 and have
 * confirmed your camera heading is accurate.
 */
public class VisionConfig {

    // ── Required ──────────────────────────────────────────────────────────────
    public final Limelight[] limelights;
    /** Axis-aligned field boundary used to reject out-of-field poses. */
    public RectangularPoseArea fieldArea = null;

    // ── Measurement quality ───────────────────────────────────────────────────
    public double xyStdDevCoefficient = 0.005;
    public double thetaStdDevCoefficient = 9999999.0;
    public double maxAmbiguity = 0.3;
    public double maxTagDistance = 4.0;
    public double maxAngularVelocityDegPerSec = 540.0;
    public double maxLinearVelocityMetersPerSec = 3.5;
    /** If true, single-tag ambiguity rejection is bypassed (e.g. via dashboard toggle). */
    public boolean useDisableAmbiguityRejection = false;

    // ── Simulation ────────────────────────────────────────────────────────────
    public AprilTagFieldLayout fieldLayout = null;
    /** Horizontal FOV in degrees — used for sim visibility checks and auto-crop coordinate conversion. */
    public double simulatedFOVDegrees = 63.3;
    /** Vertical FOV in degrees — used for auto-crop coordinate conversion. Default matches LL3/LL4. */
    public double verticalFOVDegrees = 49.7;
    public Supplier<Pose2d> groundTruthPoseSupplier = null;

    public VisionConfig(Limelight[] limelights) {
        this.limelights = limelights;
    }

    public VisionConfig withFieldArea(RectangularPoseArea fieldArea) {
        this.fieldArea = fieldArea;
        return this;
    }

    public VisionConfig withXYStdDevCoefficient(double coefficient) {
        this.xyStdDevCoefficient = coefficient;
        return this;
    }

    public VisionConfig withThetaStdDevCoefficient(double coefficient) {
        this.thetaStdDevCoefficient = coefficient;
        return this;
    }

    public VisionConfig withMaxAmbiguity(double maxAmbiguity) {
        this.maxAmbiguity = maxAmbiguity;
        return this;
    }

    public VisionConfig withMaxTagDistance(double meters) {
        this.maxTagDistance = meters;
        return this;
    }

    public VisionConfig withMaxAngularVelocity(double degPerSec) {
        this.maxAngularVelocityDegPerSec = degPerSec;
        return this;
    }

    public VisionConfig withMaxLinearVelocity(double metersPerSec) {
        this.maxLinearVelocityMetersPerSec = metersPerSec;
        return this;
    }

    public VisionConfig withDisableAmbiguityRejection(boolean disable) {
        this.useDisableAmbiguityRejection = disable;
        return this;
    }

    public VisionConfig withFieldLayout(AprilTagFieldLayout layout) {
        this.fieldLayout = layout;
        return this;
    }

    public VisionConfig withSimulatedFOVDegrees(double fovDegrees) {
        this.simulatedFOVDegrees = fovDegrees;
        return this;
    }

    public VisionConfig withVerticalFOVDegrees(double fovDegrees) {
        this.verticalFOVDegrees = fovDegrees;
        return this;
    }

    public VisionConfig withGroundTruthPoseSupplier(Supplier<Pose2d> supplier) {
        this.groundTruthPoseSupplier = supplier;
        return this;
    }
}
