package com.teamscreamrobotics.power;

public enum PowerPriority {
    /** Never throttled. Drivetrains, safety-critical systems. */
    CRITICAL,
    /** Minimal degradation only. Primary competition-affecting mechanisms. */
    HIGH,
    /** Progressively throttled under demand or voltage sag. Secondary mechanisms. */
    MEDIUM,
    /** Cut first when budget is exhausted. Non-essential accessories. */
    LOW
}
