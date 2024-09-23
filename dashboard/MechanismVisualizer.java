package com.SCREAMLib.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.constants.SimConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BiConsumer;
import lombok.Setter;

public class MechanismVisualizer extends SubsystemBase {

  public Mechanism2d MEASURED_MECHANISM = SimConstants.MEASURED_MECHANISM;
  public Mechanism2d SETPOINT_MECHANISM = SimConstants.SETPOINT_MECHANISM;

  @Setter private boolean enabled = true;
  private BiConsumer<Mechanism2d, Mechanism2d> telemetryCons;

  private final ArrayList<Mechanism> mechanisms;

  public MechanismVisualizer(
      BiConsumer<Mechanism2d, Mechanism2d> telemetryConsumer, Mechanism... mechanisms) {
    this.telemetryCons = telemetryConsumer;
    this.mechanisms = new ArrayList<>(Arrays.asList(mechanisms));
    this.mechanisms.forEach((mech) -> mech.initialize(MEASURED_MECHANISM, SETPOINT_MECHANISM));
  }

  @Override
  public void periodic() {
    if (enabled) {
      mechanisms.forEach(Mechanism::update);
      telemetryCons.accept(MEASURED_MECHANISM, SETPOINT_MECHANISM);
    }
  }
}
