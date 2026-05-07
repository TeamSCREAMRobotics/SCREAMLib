package com.teamscreamrobotics.superstructure;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.*;

/**
 * Immutable-after-construction graph of superstructure positions and safe transitions.
 *
 * <p>Build once at robot init:
 * <pre>
 *   SuperstructureGraph graph = new SuperstructureGraph()
 *       .registerMechanism(elevatorAdapter, 0.5)
 *       .registerMechanism(armAdapter,      0.02)
 *       .addNode(STOW).addNode(INTAKE)
 *       .addEdge(STOW, INTAKE);
 * </pre>
 *
 * <p>Validation is performed lazily on the first A* call and catches:
 * <ul>
 *   <li>Positions that reference an adapter not registered with this graph</li>
 *   <li>Duplicate node names</li>
 *   <li>Island nodes (no edges)</li>
 * </ul>
 */
public class SuperstructureGraph {

    /** adapter → time-per-unit coefficient (seconds per natural unit). */
    private final Map<MechanismAdapter, Double> registeredAdapters = new LinkedHashMap<>();
    private final Map<String, SuperstructureNode> nodes = new LinkedHashMap<>();
    private final Map<SuperstructureNode, List<SuperstructureEdge>> adjacency = new LinkedHashMap<>();

    private boolean validated = false;

    // ── Construction API ──────────────────────────────────────────────────────

    /**
     * Registers a mechanism and its time-per-unit coefficient used for edge cost and heuristic.
     *
     * @param adapter         mechanism adapter — this exact object reference must be used in
     *                        {@link SuperstructurePosition.Builder#set} calls
     * @param timeCoefficient seconds per natural unit (e.g. 0.5 s/m for elevator, 0.02 s/deg for arm)
     */
    public SuperstructureGraph registerMechanism(MechanismAdapter adapter, double timeCoefficient) {
        registeredAdapters.put(adapter, timeCoefficient);
        validated = false;
        return this;
    }

    public SuperstructureGraph addNode(SuperstructureNode node) {
        if (nodes.containsKey(node.name)) {
            throw new IllegalStateException("SuperstructureGraph: duplicate node name '" + node.name + "'");
        }
        nodes.put(node.name, node);
        adjacency.put(node, new ArrayList<>());
        validated = false;
        return this;
    }

    /** Adds an undirected edge between {@code a} and {@code b} (edges in both directions, same cost). */
    public SuperstructureGraph addEdge(SuperstructureNode a, SuperstructureNode b,
                                       EdgeCostModifier... modifiers) {
        autoAddNode(a);
        autoAddNode(b);
        double cost = computeBaseCost(a, b);
        adjacency.get(a).add(new SuperstructureEdge(a, b, cost, modifiers));
        adjacency.get(b).add(new SuperstructureEdge(b, a, cost, modifiers));
        validated = false;
        return this;
    }

    /** Adds a directed edge from {@code from} to {@code to} only. */
    public SuperstructureGraph addDirectedEdge(SuperstructureNode from, SuperstructureNode to,
                                               EdgeCostModifier... modifiers) {
        autoAddNode(from);
        autoAddNode(to);
        double cost = computeBaseCost(from, to);
        adjacency.get(from).add(new SuperstructureEdge(from, to, cost, modifiers));
        validated = false;
        return this;
    }

    // ── Package-private accessors for A* and state machine ────────────────────

    List<SuperstructureEdge> getEdges(SuperstructureNode node) {
        return adjacency.getOrDefault(node, Collections.emptyList());
    }

    Collection<SuperstructureNode> getAllNodes() {
        return nodes.values();
    }

    /** Returns registered adapters in registration order. */
    List<MechanismAdapter> getRegisteredAdapters() {
        return new ArrayList<>(registeredAdapters.keySet());
    }

    double getTimeCoefficient(MechanismAdapter adapter) {
        return registeredAdapters.getOrDefault(adapter, 1.0);
    }

    Map<MechanismAdapter, Double> getRegisteredAdaptersWithCoefficients() {
        return Collections.unmodifiableMap(registeredAdapters);
    }

    SuperstructureNode getNode(String name) {
        return nodes.get(name);
    }

    // ── Validation ────────────────────────────────────────────────────────────

    /** Validates graph consistency. Called automatically before the first A* search. */
    public SuperstructureGraph validate() {
        for (SuperstructureNode node : nodes.values()) {
            for (MechanismAdapter adapter : node.position.mechanismSetpoints().keySet()) {
                if (!registeredAdapters.containsKey(adapter)) {
                    throw new IllegalStateException(
                            "SuperstructureGraph: position '" + node.name
                            + "' references adapter '" + adapter.getName()
                            + "' that was not registered with this graph. "
                            + "Ensure the same adapter object is used in both "
                            + "registerMechanism() and SuperstructurePosition.Builder.set().");
                }
            }
        }

        for (Map.Entry<SuperstructureNode, List<SuperstructureEdge>> entry : adjacency.entrySet()) {
            if (entry.getValue().isEmpty()) {
                DriverStation.reportWarning(
                        "SuperstructureGraph: node '" + entry.getKey().name
                        + "' has no edges and can never be reached or departed.", false);
            }
        }

        validated = true;
        return this;
    }

    void validateOnce() {
        if (!validated) validate();
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    private void autoAddNode(SuperstructureNode node) {
        if (!nodes.containsKey(node.name)) {
            nodes.put(node.name, node);
            adjacency.put(node, new ArrayList<>());
        }
    }

    private double computeBaseCost(SuperstructureNode a, SuperstructureNode b) {
        double cost = 0.0;
        for (Map.Entry<MechanismAdapter, Double> entry : registeredAdapters.entrySet()) {
            MechanismAdapter adapter = entry.getKey();
            double coeff = entry.getValue();
            double va = a.position.getSetpoint(adapter);
            double vb = b.position.getSetpoint(adapter);
            cost += Math.abs(vb - va) * coeff;
        }
        return cost;
    }
}
