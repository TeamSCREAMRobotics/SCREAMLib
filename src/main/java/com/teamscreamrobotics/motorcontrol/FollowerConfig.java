package com.teamscreamrobotics.motorcontrol;

/**
 * Describes a single follower TalonFX that mirrors the master motor's output.
 *
 * @param canId   CAN device ID of the follower TalonFX.
 * @param canbus  CAN bus name; empty string selects the default bus.
 * @param oppose  {@code true} to run opposite the master (e.g. motors mounted facing each other).
 */
public record FollowerConfig(int canId, String canbus, boolean oppose) {

    /** Default CAN bus, not opposed. */
    public FollowerConfig(int canId) {
        this(canId, "", false);
    }

    /** Default CAN bus. */
    public FollowerConfig(int canId, boolean oppose) {
        this(canId, "", oppose);
    }
}
