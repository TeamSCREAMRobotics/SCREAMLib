package com.teamscreamrobotics.sim;

/** Callback invoked after each simulation step with the updated position and velocity. */
@Deprecated(since = "2027")
@FunctionalInterface
public interface SimStateCallback {
  void accept(double position, double velocity);
}
