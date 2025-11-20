package com.teamscreamrobotics.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BiConsumer;

public class MechanismVisualizer extends SubsystemBase {

  public Mechanism2d MEASURED_MECHANISM;
  public Mechanism2d SETPOINT_MECHANISM;

  private boolean enabled = true;
  private BiConsumer<Mechanism2d, Mechanism2d> telemetryCons;

  private final ArrayList<Mechanism> mechanisms;

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
