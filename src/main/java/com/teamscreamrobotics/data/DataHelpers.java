package com.teamscreamrobotics.data;

/** Miscellaneous data types and functional interfaces used across the library. */
public class DataHelpers {
  /** A functional interface for a consumer accepting three arguments. */
  @FunctionalInterface
  public static interface TriConsumer<T1, T2, T3> {
    void accept(T1 t1, T2 t2, T3 t3);
  }

  /** A simple (height, width) size pair. */
  public static record Dimensions(double height, double width) {}

  /** Associates a log key with a typed value for structured telemetry output. */
  public static record LoggedOutput<T>(String key, T value){}
}
