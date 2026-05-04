package com.teamscreamrobotics.power;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

/**
 * Static power arbitration layer
 *
 * <h2>Usage</h2>
 * <pre>
 *   // In robotPeriodic(), before CommandScheduler.run():
 *   PowerManager.update();
 *
 *   // Optionally, to enable the reactive (PDH-based) layer:
 *   PowerManager.configure(new PowerDistribution(1, PowerDistribution.ModuleType.kRev));
 * </pre>
 *
 * <p>{@link com.teamscreamrobotics.motorcontrol.TalonFXWrapper} registers itself
 * automatically on construction, so no manual registration is required.
 *
 * <h2>Behavior when PDH is not configured</h2>
 * <p>Brownout detection is disabled. The predictive layer still operates on
 * {@code estimateDemandWatts()} from each consumer against a nominal 12.5V budget.
 * Constraints are still debounced and distributed normally.
 *
 * <h2>Update ordering</h2>
 * <p>{@code update()} must run before {@code CommandScheduler.run()}.
 * Constraints emitted this cycle are applied to commands in the same cycle
 * (1-cycle lag from measurement to application is acceptable at 20ms).
 */
public final class PowerManager {

    private PowerManager() {}

    // ── Thresholds ────────────────────────────────────────────────────────
    private static final double BROWNOUT_CRITICAL_V = 7.5;
    private static final double BROWNOUT_WARNING_V  = 8.5;
    private static final double NOMINAL_VOLTAGE     = 12.5;
    private static final double MAX_ROBOT_CURRENT_A = 200.0;

    // ── EMA coefficients ──────────────────────────────────────────────────
    // Heavy voltage filter (10-cycle lag) avoids reacting to single-sample noise.
    // Lighter current filter (5-cycle lag) responds faster to demand surges.
    private static final double VOLTAGE_ALPHA = 0.1;
    private static final double CURRENT_ALPHA = 0.2;

    // ── Debounce: prevents tier flickering under transient demand spikes ──
    private static final int DEBOUNCE_CYCLES = 3;

    // ── Battery health warning ────────────────────────────────────────────
    // Fires only outside FMS (pit/practice) and only when current draw is low
    // enough that voltage sag from load isn't the cause.
    private static final double LOW_BATTERY_WARNING_V  = 12.0;
    private static final double IDLE_CURRENT_THRESHOLD_A = 20.0;

    // ── Pre-built brownout constraints (avoid allocation in hot path) ─────
    private static final PowerConstraint BROWNOUT_HIGH = new PowerConstraint(0.5, 0.5, 0.45, false);
    private static final PowerConstraint WARNING_HIGH  = new PowerConstraint(0.8, 0.8, 0.70, false);
    private static final PowerConstraint WARNING_MEDIUM  = new PowerConstraint(0.5, 0.5, 0.45, false);

    // ── Mutable state ─────────────────────────────────────────────────────
    private static PowerDistribution pdh = null;
    private static final List<PowerConsumer> consumers = new ArrayList<>();

    private static double filteredVoltage  = NOMINAL_VOLTAGE;
    private static double filteredCurrent  = 0.0;
    private static int    debounceCount    = 0;
    private static boolean constrained     = false;
    private static boolean lowBatteryWarned = false;

    // ── Public API ────────────────────────────────────────────────────────

    /**
     * Optionally configure a PDH/PDP to enable the reactive (current + voltage
     * based) layer. Without this, the system operates on demand estimates only.
     */
    public static void configure(PowerDistribution pdh) {
        PowerManager.pdh = pdh;
    }

    /**
     * Register a consumer. Called automatically by {@code TalonFXWrapper} —
     * teams using custom hardware wrappers may call this manually.
     */
    public static void register(PowerConsumer consumer) {
        consumers.add(consumer);
    }

    /**
     * Run one arbitration cycle. Call from {@code robotPeriodic()} before
     * {@code CommandScheduler.run()}.
     */
    public static void update() {
        // 1. Read and filter PDH if configured
        if (pdh != null) {
            filteredVoltage = VOLTAGE_ALPHA * pdh.getVoltage()
                            + (1 - VOLTAGE_ALPHA) * filteredVoltage;
            filteredCurrent = CURRENT_ALPHA * pdh.getTotalCurrent()
                            + (1 - CURRENT_ALPHA) * filteredCurrent;
        }

        // 2. Sum demand estimates by priority tier
        double criticalW = 0, highW = 0, mediumW = 0, lowW = 0;
        for (PowerConsumer c : consumers) {
            double w = c.estimateDemandWatts();
            switch (c.getPowerPriority()) {
                case CRITICAL -> criticalW += w;
                case HIGH     -> highW     += w;
                case MEDIUM   -> mediumW   += w;
                case LOW      -> lowW      += w;
            }
        }

        // 3. Compute per-tier constraints
        boolean isBrownout = filteredVoltage < BROWNOUT_WARNING_V;
        PowerConstraint critConstraint, highConstraint, medConstraint, lowConstraint;

        if (filteredVoltage < BROWNOUT_CRITICAL_V) {
            // Emergency — protect against collapse immediately (no debounce)
            critConstraint = PowerConstraint.UNCONSTRAINED;
            highConstraint = BROWNOUT_HIGH;
            medConstraint  = PowerConstraint.STOP;
            lowConstraint  = PowerConstraint.STOP;
        } else if (isBrownout) {
            critConstraint = PowerConstraint.UNCONSTRAINED;
            highConstraint = WARNING_HIGH;
            medConstraint  = WARNING_MEDIUM;
            lowConstraint  = PowerConstraint.STOP;
        } else {
            // Normal: demand-based allocation against the robot's power budget
            double budget = filteredVoltage * MAX_ROBOT_CURRENT_A;

            critConstraint = PowerConstraint.UNCONSTRAINED;

            double highBudget = budget - criticalW;
            highConstraint = allocate(highW, highBudget, 0.6);

            double medBudget = Math.max(0, highBudget - highW);
            medConstraint = allocate(mediumW, medBudget, 0.25);

            double lowBudget = Math.max(0, medBudget - mediumW);
            lowConstraint = allocate(lowW, lowBudget, 0.0);
        }

        // 4. Debounce demand-based constraints (brownout bypasses this)
        boolean wantsConstraint = !highConstraint.isUnconstrained()
                || !medConstraint.isUnconstrained()
                || !lowConstraint.isUnconstrained();

        if (wantsConstraint) {
            debounceCount = Math.min(debounceCount + 1, DEBOUNCE_CYCLES + 1);
        } else {
            debounceCount = Math.max(debounceCount - 1, 0);
        }
        // Enter constrained state after sustained demand; exit immediately when clear
        if (debounceCount >= DEBOUNCE_CYCLES) constrained = true;
        if (debounceCount == 0)              constrained = false;

        if (!constrained && !isBrownout) {
            highConstraint = medConstraint = lowConstraint = PowerConstraint.UNCONSTRAINED;
        }

        // 5. Distribute constraints to consumers
        for (PowerConsumer c : consumers) {
            PowerConstraint constraint = switch (c.getPowerPriority()) {
                case CRITICAL -> critConstraint;
                case HIGH     -> highConstraint;
                case MEDIUM   -> medConstraint;
                case LOW      -> lowConstraint;
            };
            c.applyPowerConstraint(constraint);
        }

        // 6. AKit telemetry
        Logger.recordOutput("PowerManager/FilteredVoltage",     filteredVoltage);
        Logger.recordOutput("PowerManager/FilteredCurrentAmps", filteredCurrent);
        Logger.recordOutput("PowerManager/BudgetWatts",         filteredVoltage * MAX_ROBOT_CURRENT_A);
        Logger.recordOutput("PowerManager/CriticalDemandW",     criticalW);
        Logger.recordOutput("PowerManager/HighDemandW",         highW);
        Logger.recordOutput("PowerManager/MediumDemandW",       mediumW);
        Logger.recordOutput("PowerManager/LowDemandW",          lowW);
        Logger.recordOutput("PowerManager/Constrained",         constrained);
        Logger.recordOutput("PowerManager/Brownout",            isBrownout);
        Logger.recordOutput("PowerManager/HighVelocityCap",     highConstraint.velocityCap);
        Logger.recordOutput("PowerManager/MediumVelocityCap",   medConstraint.velocityCap);

        // 7. Low battery warning (pit/practice only, idle load only)
        boolean batteryLow = filteredVoltage < LOW_BATTERY_WARNING_V
                && filteredCurrent < IDLE_CURRENT_THRESHOLD_A
                && !DriverStation.isFMSAttached();
        if (batteryLow && !lowBatteryWarned) {
            DriverStation.reportWarning(
                    String.format("Battery voltage low (%.1fV at idle) — consider changing the battery",
                            filteredVoltage),
                    false);
            lowBatteryWarned = true;
        } else if (!batteryLow) {
            lowBatteryWarned = false;
        }

        // Per-consumer PDH channel current (logged only when channels are mapped)
        if (pdh != null) {
            for (PowerConsumer c : consumers) {
                int[] channels = c.getPDHChannels();
                if (channels.length == 0) continue;
                double totalAmps = 0;
                for (int ch : channels) totalAmps += pdh.getCurrent(ch);
                Logger.recordOutput("PowerManager/Consumers/" + c.getConsumerName() + "Amps", totalAmps);
            }
        }
    }

    /**
     * Computes a constraint for a tier given its demand and available budget.
     *
     * @param demand     watts requested by this tier
     * @param budget     watts available to this tier
     * @param floorRatio minimum allowed cap ratio before STOP is issued instead
     */
    private static PowerConstraint allocate(double demand, double budget, double floorRatio) {
        if (demand <= 0 || budget >= demand) return PowerConstraint.UNCONSTRAINED;
        double ratio = budget / demand;
        if (ratio < floorRatio) return PowerConstraint.STOP;
        // FF cap is slightly tighter than velocity: gravity feedforward is the
        // marginal watts that tip an already-loaded mechanism into overload.
        return new PowerConstraint(ratio, ratio, ratio * 0.9, false);
    }
}
