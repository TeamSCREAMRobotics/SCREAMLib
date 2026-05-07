package com.teamscreamrobotics.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

import java.util.*;

/**
 * Coordinates multiple mechanisms through a collision-avoidance graph using A* path planning.
 *
 * <p>Call {@link #periodic()} from the owning subsystem's {@code periodic()} method each loop.
 * Call {@link #setGoal} to request a new superstructure configuration.
 *
 * <h2>Usage example</h2>
 * <pre>
 *   // 1. Adapters stored as fields - these are the identifiers
 *   ElevatorAdapter elevatorAdapter = new ElevatorAdapter("elevator", elevator);
 *   ArmAdapter      armAdapter      = new ArmAdapter("arm", arm);
 *   PivotAdapter    wristAdapter    = new PivotAdapter("wrist", wrist);
 *
 *   // 2. Define nodes using typed setpoints
 *   SuperstructureNode STOW = new SuperstructureNode("STOW",
 *       SuperstructurePosition.builder()
 *           .set(elevatorAdapter, Meters.of(0.0))
 *           .set(armAdapter,      Degrees.of(-10.0))
 *           .set(wristAdapter,    Degrees.of(0.0))
 *           .build());
 *
 *   SuperstructureNode SCORE_L3 = new SuperstructureNode("SCORE_L3",
 *       SuperstructurePosition.builder()
 *           .set(elevatorAdapter, Meters.of(0.8))
 *           .set(armAdapter,      Degrees.of(85.0))
 *           .set(wristAdapter,    Degrees.of(45.0))
 *           .build());
 *
 *   SuperstructureNode SAFE_PASS = new SuperstructureNode("SAFE_PASS",
 *       SuperstructurePosition.builder()
 *           .set(elevatorAdapter, Meters.of(0.3))
 *           .set(armAdapter,      Degrees.of(0.0))
 *           .set(wristAdapter,    Degrees.of(0.0))
 *           .build());
 *
 *   // 3. Build graph — same adapter objects used in registerMechanism and .set()
 *   ProximityModifier reefModifier = new ProximityModifier(
 *       "ReefProximity", new Translation2d(4.5, 4.0), 1.0, 50.0, drivetrain::getPose);
 *
 *   SuperstructureGraph graph = new SuperstructureGraph()
 *       .registerMechanism(elevatorAdapter, 0.5)
 *       .registerMechanism(armAdapter,      0.02)
 *       .registerMechanism(wristAdapter,    0.015)
 *       .addNode(STOW).addNode(SCORE_L3).addNode(SAFE_PASS)
 *       .addEdge(STOW,      SAFE_PASS)
 *       .addEdge(SAFE_PASS, SCORE_L3, reefModifier);
 *
 *   // 4. Create state machine
 *   SuperstructureStateMachine superstructure = new SuperstructureStateMachine(graph);
 *
 *   // 5. In subsystem periodic()
 *   superstructure.periodic();
 *   superstructure.setGoal(SCORE_L3);
 *   // A*: STOW -&gt; SAFE_PASS -&gt; SCORE_L3
 *   // Near the reef, SAFE_PASS -&gt; SCORE_L3 costs more — A* may reroute.
 * </pre>
 */
public class SuperstructureStateMachine {

    private enum State { IDLE, PLANNING, TRANSITIONING, AT_GOAL }

    /** If a transition has not completed within this many seconds, advance anyway to avoid deadlock. */
    public double transitionTimeoutSeconds = 3.0;

    private final SuperstructureGraph graph;
    private final List<MechanismAdapter> adapters;

    private State state = State.IDLE;
    private SuperstructureNode currentNode = null;
    private SuperstructureNode goalNode = null;
    private final LinkedList<SuperstructureNode> remainingPath = new LinkedList<>();
    private final Map<MechanismAdapter, Double> currentSetpoints = new LinkedHashMap<>();
    private double transitionStartTime = 0.0;

    public SuperstructureStateMachine(SuperstructureGraph graph) {
        this.graph = graph;
        this.adapters = graph.getRegisteredAdapters();
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Commands the superstructure toward {@code goal}.
     * Safe to call every loop with the same goal — idempotent while already moving toward it.
     */
    public void setGoal(SuperstructureNode goal) {
        if (goal.equals(goalNode) && (state == State.TRANSITIONING || state == State.AT_GOAL)) return;
        goalNode = goal;
        state = State.PLANNING;
    }

    /** Convenience overload that looks up a node by name. Logs a warning if not found. */
    public void setGoal(String goalName) {
        SuperstructureNode node = graph.getNode(goalName);
        if (node == null) {
            DriverStation.reportWarning(
                    "SuperstructureStateMachine: unknown goal node '" + goalName + "'", false);
            return;
        }
        setGoal(node);
    }

    public SuperstructureNode getCurrentNode()       { return currentNode; }
    public SuperstructureNode getGoalNode()          { return goalNode; }
    public boolean atGoal()                          { return state == State.AT_GOAL; }
    public boolean isTransitioning()                 { return state == State.TRANSITIONING; }

    /** Returns the planned path from the current node to the goal, including intermediate nodes. */
    public List<SuperstructureNode> getCurrentPath() {
        List<SuperstructureNode> path = new ArrayList<>();
        if (currentNode != null) path.add(currentNode);
        path.addAll(remainingPath);
        return path;
    }

    /**
     * Advances the state machine. Must be called from the owning subsystem's {@code periodic()}.
     */
    public void periodic() {
        switch (state) {
            case IDLE          -> {}
            case PLANNING      -> handlePlanning();
            case TRANSITIONING -> handleTransitioning();
            case AT_GOAL       -> holdGoal();
        }
        logTelemetry();
    }

    // ── State handlers ────────────────────────────────────────────────────────

    private void handlePlanning() {
        SuperstructureNode nearest = findNearestNode();
        if (nearest == null) {
            DriverStation.reportWarning("SuperstructureStateMachine: graph has no nodes", false);
            state = State.IDLE;
            return;
        }

        List<SuperstructureNode> path = SuperstructureAstar.findPath(nearest, goalNode, graph);

        if (path.isEmpty()) {
            state = State.IDLE;
            return;
        }

        remainingPath.clear();
        remainingPath.addAll(path);
        currentNode = remainingPath.removeFirst(); // first node is where we are

        if (remainingPath.isEmpty()) {
            state = State.AT_GOAL;
            holdGoal();
            return;
        }

        commandNextNode();
        state = State.TRANSITIONING;
        transitionStartTime = Timer.getFPGATimestamp();
    }

    private void handleTransitioning() {
        SuperstructureNode target = remainingPath.peekFirst();
        if (target == null) {
            state = State.AT_GOAL;
            return;
        }

        // Re-command each cycle so MotionMagic stays targeted even across control-loop resets
        commandToNode(target);

        boolean allReady = adapters.stream().allMatch(a ->
                a.atSetpoint(target.position.getSetpoint(a), a.getDefaultTolerance()));

        if (!allReady && Timer.getFPGATimestamp() - transitionStartTime > transitionTimeoutSeconds) {
            DriverStation.reportWarning(
                    "SuperstructureStateMachine: transition to '" + target.name
                    + "' timed out after " + transitionTimeoutSeconds + " s — advancing", false);
            allReady = true;
        }

        if (allReady) {
            currentNode = remainingPath.removeFirst();
            if (remainingPath.isEmpty()) {
                state = State.AT_GOAL;
            } else {
                commandNextNode();
                transitionStartTime = Timer.getFPGATimestamp();
            }
        }
    }

    private void holdGoal() {
        if (goalNode == null) return;
        commandToNode(goalNode);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private void commandNextNode() {
        SuperstructureNode next = remainingPath.peekFirst();
        if (next != null) commandToNode(next);
    }

    private void commandToNode(SuperstructureNode node) {
        for (MechanismAdapter adapter : adapters) {
            double setpoint = node.position.getSetpoint(adapter);
            currentSetpoints.put(adapter, setpoint);
            adapter.setSetpoint(setpoint);
        }
    }

    /**
     * Returns the graph node with the minimum weighted distance from current mechanism positions.
     * Uses time coefficients as weights so the result is in units of approximate travel time.
     */
    private SuperstructureNode findNearestNode() {
        return graph.getAllNodes().stream()
                .min(Comparator.comparingDouble(node -> {
                    double error = 0.0;
                    for (MechanismAdapter adapter : adapters) {
                        double setpoint = node.position.getSetpoint(adapter);
                        double coeff = graph.getTimeCoefficient(adapter);
                        error += Math.abs(adapter.getCurrentValue() - setpoint) * coeff;
                    }
                    return error;
                }))
                .orElse(null);
    }

    // ── Telemetry ─────────────────────────────────────────────────────────────

    private void logTelemetry() {
        Logger.recordOutput("Superstructure/State",       state.name());
        Logger.recordOutput("Superstructure/CurrentNode", currentNode != null ? currentNode.name : "Unknown");
        Logger.recordOutput("Superstructure/GoalNode",    goalNode    != null ? goalNode.name    : "None");
        Logger.recordOutput("Superstructure/AtGoal",      atGoal());
        Logger.recordOutput("Superstructure/PathLength",  getCurrentPath().size());
        Logger.recordOutput("Superstructure/PathNodes",
                getCurrentPath().stream().map(n -> n.name).toArray(String[]::new));

        for (MechanismAdapter adapter : adapters) {
            String base = "Superstructure/Mechanisms/" + adapter.getName() + "/";
            Logger.recordOutput(base + "Setpoint",   currentSetpoints.getOrDefault(adapter, 0.0));
            Logger.recordOutput(base + "AtSetpoint", adapter.atSetpoint());
        }
    }
}
