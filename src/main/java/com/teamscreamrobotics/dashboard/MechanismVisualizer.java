package com.teamscreamrobotics.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BiConsumer;

/**
 * A WPILib subsystem that periodically updates one or more {@link Mechanism}s and publishes
 * their measured and setpoint {@link Mechanism2d} widgets via a telemetry consumer.
 */
public class MechanismVisualizer extends SubsystemBase {

  public Mechanism2d MEASURED_MECHANISM;
  public Mechanism2d SETPOINT_MECHANISM;

  private boolean enabled = true;
  private BiConsumer<Mechanism2d, Mechanism2d> telemetryCons;

  private final ArrayList<Mechanism> mechanisms;

  /**
   * Creates the visualizer and initializes all mechanisms.
   *
   * @param measuredMechanism  the {@link Mechanism2d} widget for actual state (typically logged red)
   * @param setpointMechanism  the {@link Mechanism2d} widget for setpoint state (typically logged green)
   * @param telemetryConsumer  called each periodic cycle with (measured, setpoint) to publish data
   * @param mechanisms         one or more {@link Mechanism}s to update and display
   */
  public MechanismVisualizer(
      Mechanism2d measuredMechanism,
      Mechanism2d setpointMechanism,
      BiConsumer<Mechanism2d, Mechanism2d> telemetryConsumer,
      Mechanism... mechanisms) {
    MEASURED_MECHANISM = measuredMechanism;
    SETPOINT_MECHANISM = setpointMechanism;
    this.telemetryCons = telemetryConsumer;
    this.mechanisms = new ArrayList<>(Arrays.asList(mechanisms));
    this.mechanisms.forEach((mech) -> mech.initialize(MEASURED_MECHANISM, SETPOINT_MECHANISM));
  }

  /** Enables or disables periodic updates. Disable to suppress logging overhead when not needed. */
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  @Override
  public void periodic() {
    if (enabled) {
      mechanisms.forEach(Mechanism::update);
      telemetryCons.accept(MEASURED_MECHANISM, SETPOINT_MECHANISM);
    }
  }
}
