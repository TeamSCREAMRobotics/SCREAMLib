package com.teamscreamrobotics.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;

public class SimulationThread {

  private String name;
  private SimInterface simInterface;
  private Notifier simNotifier = null;
  private double lastSimTime;

  private double deltaTime;

  private DoubleSupplier simVoltage = () -> 0.0;

  private BiConsumer<Double, Double> stateConsumer;

  private boolean useSeparateThread;

  private double periodSec;
  private boolean limitVoltage;

  public SimulationThread(
      TalonFXSubsystemSimConstants constants,
      BiConsumer<Double, Double> stateConsumer,
      double periodSec,
      String name) {
    this.name = name;
    this.simInterface = constants.sim();
    this.useSeparateThread = constants.useSeparateThread();
    this.limitVoltage = constants.limitVoltage();
    this.stateConsumer = stateConsumer;
    this.periodSec = periodSec;
    if (useSeparateThread) {
      startSimThread(name);
    }
  }

  public void setSimVoltage(DoubleSupplier simVoltage) {
    this.simVoltage =
        () ->
            limitVoltage
                ? MathUtil.clamp(simVoltage.getAsDouble(), -12, 12)
                : simVoltage.getAsDouble();
  }

  public DoubleSupplier getSimVoltage() {
    return simVoltage;
  }

  public void update() {
    if (useSeparateThread) {
      DriverStation.reportError(
          "Simulation thread: " + name + " | Do not call update if using separate thread!", true);
      return;
    }
    final double currentTime = Timer.getFPGATimestamp();
    deltaTime = currentTime - lastSimTime;
    lastSimTime = currentTime;

    simInterface.update(deltaTime);
    simInterface.setInputVoltage(
        simVoltage.getAsDouble());
    stateConsumer.accept(simInterface.getPosition(), simInterface.getVelocity());
  }

  public void startSimThread(String name) {
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Timer.getFPGATimestamp();
              deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              simInterface.update(deltaTime);
              simInterface.setInputVoltage(
                  simVoltage.getAsDouble());
              stateConsumer.accept(simInterface.getPosition(), simInterface.getVelocity());
            });
    simNotifier.setName(name);
    simNotifier.startPeriodic(periodSec);
  }
}
