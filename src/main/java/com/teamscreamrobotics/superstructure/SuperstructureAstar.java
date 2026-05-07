package com.teamscreamrobotics.superstructure;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.*;

/** Package-private A* implementation over a {@link SuperstructureGraph}. */
class SuperstructureAstar {

    private SuperstructureAstar() {}

    /**
     * Finds the minimum-cost path from {@code start} to {@code goal}.
     *
     * <p>Returns a list containing just {@code start} if start == goal.
     * Returns an empty list if no path exists, and logs a DriverStation warning.
     *
     * <p>Edge costs are evaluated at search time, so proximity modifiers reflect the
     * robot's current position when the path is requested.
     */
    static List<SuperstructureNode> findPath(
            SuperstructureNode start,
            SuperstructureNode goal,
            SuperstructureGraph graph) {

        graph.validateOnce();

        if (start.equals(goal)) return List.of(start);

        Map<SuperstructureNode, Double> gScore = new HashMap<>();
        Map<SuperstructureNode, Double> fScore = new HashMap<>();
        Map<SuperstructureNode, SuperstructureNode> cameFrom = new HashMap<>();
        Set<SuperstructureNode> closed = new HashSet<>();

        // Priority queue ordered by fScore; fScore is updated before every insertion
        PriorityQueue<SuperstructureNode> open = new PriorityQueue<>(
                Comparator.comparingDouble(n -> fScore.getOrDefault(n, Double.MAX_VALUE)));

        gScore.put(start, 0.0);
        fScore.put(start, heuristic(start, goal, graph));
        open.add(start);

        while (!open.isEmpty()) {
            SuperstructureNode current = open.poll();

            if (closed.contains(current)) continue; // stale entry from lazy re-insertion
            closed.add(current);

            if (current.equals(goal)) return reconstructPath(cameFrom, current);

            for (SuperstructureEdge edge : graph.getEdges(current)) {
                if (closed.contains(edge.to)) continue;

                double tentativeG = gScore.getOrDefault(current, Double.MAX_VALUE)
                        + edge.getTotalCost();

                if (tentativeG < gScore.getOrDefault(edge.to, Double.MAX_VALUE)) {
                    cameFrom.put(edge.to, current);
                    gScore.put(edge.to, tentativeG);
                    fScore.put(edge.to, tentativeG + heuristic(edge.to, goal, graph));
                    open.add(edge.to); // lazy re-insert; stale entry handled by closed set
                }
            }
        }

        DriverStation.reportWarning(
                "SuperstructureAstar: no path from '" + start.name + "' to '" + goal.name + "'", false);
        return Collections.emptyList();
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /**
     * Admissible heuristic: sum of |goal_setpoint - current_setpoint| * timeCoefficient
     * across all mechanisms. Never overestimates because it assumes direct travel with no stops.
     */
    private static double heuristic(SuperstructureNode node, SuperstructureNode goal,
                                     SuperstructureGraph graph) {
        double h = 0.0;
        for (Map.Entry<MechanismAdapter, Double> entry :
                graph.getRegisteredAdaptersWithCoefficients().entrySet()) {
            MechanismAdapter adapter = entry.getKey();
            double coeff = entry.getValue();
            double nodeVal = node.position.getSetpoint(adapter);
            double goalVal = goal.position.getSetpoint(adapter);
            h += Math.abs(goalVal - nodeVal) * coeff;
        }
        return h;
    }

    private static List<SuperstructureNode> reconstructPath(
            Map<SuperstructureNode, SuperstructureNode> cameFrom,
            SuperstructureNode current) {
        LinkedList<SuperstructureNode> path = new LinkedList<>();
        path.addFirst(current);
        while (cameFrom.containsKey(current)) {
            current = cameFrom.get(current);
            path.addFirst(current);
        }
        return new ArrayList<>(path);
    }
}
