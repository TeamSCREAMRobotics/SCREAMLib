package com.teamscreamrobotics.power;

/**
 * Implemented by any hardware abstraction that participates in power arbitration.
 * In SCREAMLib, {@code TalonFXWrapper} implements this automatically.
 */
public interface PowerConsumer {

    PowerPriority getPowerPriority();

    /**
     * Estimated power demand for the current cycle in watts. Used by the
     * predictive layer. The reactive layer (PDH total current) corrects for
     * estimation error, so accuracy improves allocation quality but is not
     * safety-critical.
     */
    double estimateDemandWatts();

    /**
     * Receives the computed constraint once per cycle from {@link PowerManager}.
     * Implementations store it and apply it on the next control command.
     */
    void applyPowerConstraint(PowerConstraint constraint);

    /** Human-readable identifier for telemetry. */
    String getConsumerName();

    /**
     * PDH channel indices this consumer draws from. Used for per-consumer
     * current monitoring in telemetry. Return an empty array if unmapped —
     * the system degrades gracefully to pure predictive allocation.
     */
    int[] getPDHChannels();
}
