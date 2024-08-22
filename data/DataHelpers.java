package com.SCREAMLib.data;

import com.SCREAMLib.sim.SimWrapper;
import edu.wpi.first.math.controller.PIDController;

public class DataHelpers {
  @FunctionalInterface
  public interface TriConsumer<T1, T2, T3> {
    void accept(T1 t1, T2 t2, T3 t3);
  }

  public record Dimensions(double height, double width) {}

  public record SimConstants(
      SimWrapper sim,
      PIDController simController,
      boolean useSeparateThread,
      boolean limitVoltage) {
    public SimConstants(SimWrapper sim, PIDController simController) {
      this(sim, simController, false, true);
    }
  }
}
