package com.teamscreamrobotics.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A dashboard-backed double that detects when its value changes.
 *
 * <p>Use {@link #hasChanged()} to gate expensive reconfiguration:
 * <pre>
 *   if (kP.hasChanged() | kI.hasChanged() | kD.hasChanged()) {
 *       motor.reconfigure();
 *   }
 * </pre>
 * Use {@code |} (not {@code ||}) so every {@code hasChanged()} call executes and
 * updates its cached value even when an earlier one already returned {@code true}.
 */
public class LoggedTunableNumber {

    private final String dashboardKey;
    private final double defaultValue;
    private double lastValue;

    /**
     * Creates the entry and publishes {@code defaultValue} to SmartDashboard under
     * {@code dashboardKey} if it does not already exist.
     */
    public LoggedTunableNumber(String dashboardKey, double defaultValue) {
        this.dashboardKey = dashboardKey;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;
        SmartDashboard.putNumber(dashboardKey, defaultValue);
    }

    /** Reads the current value from SmartDashboard; returns {@code defaultValue} if the entry is absent. */
    public double get() {
        return SmartDashboard.getNumber(dashboardKey, defaultValue);
    }

    /**
     * Returns {@code true} if the current value differs from the last observed value,
     * and updates the cached value. Always call on every loop cycle (do not short-circuit).
     */
    public boolean hasChanged() {
        double current = get();
        if (current != lastValue) {
            lastValue = current;
            return true;
        }
        return false;
    }
}
