package com.teamscreamrobotics.power;

/**
 * Immutable descriptor emitted by {@link PowerManager} each cycle.
 *
 * <p>Expresses how much of a consumer's own configured capability it is allowed
 * to use — not a watt budget. Positional targets are never modified; only the
 * speed at which they're reached changes.
 *
 * <p>All cap fields are in [0.0, 1.0] where 1.0 = unconstrained and 0.0 = stopped.
 */
public final class PowerConstraint {

    public static final PowerConstraint UNCONSTRAINED = new PowerConstraint(1.0, 1.0, 1.0, false);
    public static final PowerConstraint STOP          = new PowerConstraint(0.0, 0.0, 0.0, true);

    /** Fraction of configured MotionMagic cruise velocity allowed. */
    public final double velocityCap;
    /** Fraction of configured MotionMagic acceleration allowed. */
    public final double accelerationCap;
    /**
     * Fraction of calculated feedforward voltage to apply. Intentionally
     * slightly tighter than velocityCap: under load, gravity/kG feedforward
     * is the marginal current that tips a system into overload.
     */
    public final double feedforwardCap;
    /** If true the consumer must stop output entirely. */
    public final boolean fullyDisabled;

    public PowerConstraint(double velocityCap, double accelerationCap,
                           double feedforwardCap, boolean fullyDisabled) {
        this.velocityCap     = velocityCap;
        this.accelerationCap = accelerationCap;
        this.feedforwardCap  = feedforwardCap;
        this.fullyDisabled   = fullyDisabled;
    }

    public boolean isUnconstrained() {
        return !fullyDisabled && velocityCap >= 1.0 && accelerationCap >= 1.0 && feedforwardCap >= 1.0;
    }
}
