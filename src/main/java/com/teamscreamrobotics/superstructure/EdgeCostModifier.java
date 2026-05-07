package com.teamscreamrobotics.superstructure;

/**
 * Adds runtime cost to a superstructure edge.
 * Called by A* at path-planning time, so modifiers reflect conditions at the moment
 * the path is computed (not at execution time).
 */
@FunctionalInterface
public interface EdgeCostModifier {
    /**
     * @param from source node of the edge being evaluated
     * @param to   destination node of the edge being evaluated
     * @return additional cost to add to the edge's base cost (must be &ge; 0)
     */
    double getAdditionalCost(SuperstructureNode from, SuperstructureNode to);
}
