package com.teamscreamrobotics.superstructure;

import java.util.Arrays;
import java.util.List;

/** A directed edge in the collision-avoidance graph with a precomputed base cost and optional runtime modifiers. */
public class SuperstructureEdge {

    public final SuperstructureNode from;
    public final SuperstructureNode to;
    public final double baseCost;

    private final List<EdgeCostModifier> modifiers;

    SuperstructureEdge(SuperstructureNode from, SuperstructureNode to,
                       double baseCost, EdgeCostModifier... modifiers) {
        this.from = from;
        this.to = to;
        this.baseCost = baseCost;
        this.modifiers = Arrays.asList(modifiers);
    }

    /**
     * Returns the total edge cost for this cycle.
     * Modifier costs are evaluated at call time, so proximity checks reflect the current robot pose.
     */
    public double getTotalCost() {
        double total = baseCost;
        for (EdgeCostModifier m : modifiers) {
            total += m.getAdditionalCost(from, to);
        }
        return total;
    }
}
