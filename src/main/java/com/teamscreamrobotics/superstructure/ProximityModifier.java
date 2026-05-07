package com.teamscreamrobotics.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.Supplier;

/**
 * Adds a flat cost penalty when the robot is within a given radius of a field position.
 * Use this to discourage paths that are dangerous near field elements (e.g. the reef).
 */
public class ProximityModifier implements EdgeCostModifier {

    private final String name;
    private final Translation2d fieldPosition;
    private final double activationRadiusMeters;
    private final double additionalCost;
    private final Supplier<Pose2d> robotPoseSupplier;

    /**
     * @param name                  human-readable label (for debugging)
     * @param fieldPosition         field-space point to avoid near
     * @param activationRadiusMeters cost activates when robot is closer than this
     * @param additionalCost        flat cost added when within radius
     * @param robotPoseSupplier     supplies current robot pose at evaluation time
     */
    public ProximityModifier(
            String name,
            Translation2d fieldPosition,
            double activationRadiusMeters,
            double additionalCost,
            Supplier<Pose2d> robotPoseSupplier) {
        this.name = name;
        this.fieldPosition = fieldPosition;
        this.activationRadiusMeters = activationRadiusMeters;
        this.additionalCost = additionalCost;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public double getAdditionalCost(SuperstructureNode from, SuperstructureNode to) {
        Pose2d pose = robotPoseSupplier.get();
        if (pose == null) return 0.0;
        double dist = pose.getTranslation().getDistance(fieldPosition);
        return dist < activationRadiusMeters ? additionalCost : 0.0;
    }

    public String getName() {
        return name;
    }
}
