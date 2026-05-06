package com.teamscreamrobotics.sim;

/** Common interface for physics simulation backends (DC motor, elevator, arm, flywheel). */
@Deprecated(since = "2027")
public interface SimInterface {
  /**
   * Advances the simulation by {@code deltaTime} seconds.
   *
   * @param deltaTime elapsed time since the last call, in seconds
   */
  void update(double deltaTime);

  /**
   * Applies the motor input voltage to the simulation.
   *
   * @param voltage input voltage in volts
   */
  void setInputVoltage(double voltage);

  /** Returns the simulated mechanism position in rotations. */
  double getPosition();

  /** Returns the simulated mechanism velocity in rotations per second. */
  double getVelocity();
}
