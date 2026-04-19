package com.teamscreamrobotics.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;

import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;

/**
 * Drives a {@link SimInterface} at a fixed period, either inline (called from {@code periodic()})
 * or on a dedicated background thread using a WPILib {@link Notifier}.
 */
public class SimulationThread {

  private String name;
  private SimInterface simInterface;
  private Notifier simNotifier = null;
  private double lastSimTime;

  private double deltaTime;

  private DoubleSupplier simVoltage = () -> 0.0;

  private SimStateCallback stateConsumer;

  private boolean useSeparateThread;

  private double periodSec;
  private boolean limitVoltage;

  /**
   * Creates a simulation thread. If {@link TalonFXSubsystemSimConstants#useSeparateThread()} is
   * {@code true} the notifier is started immediately.
   *
   * @param constants     simulation constants from the subsystem configuration
   * @param stateConsumer called with {@code (position, velocity)} after each sim step (no boxing)
   * @param periodSec     update period in seconds
   * @param name          thread/notifier name for diagnostics
   */
  public SimulationThread(
      TalonFXSubsystemSimConstants constants,
      SimStateCallback stateConsumer,
      double periodSec,
      String name) {
    this.name = name;
    this.simInterface = constants.sim();
    this.useSeparateThread = constants.useSeparateThread();
    this.limitVoltage = constants.limitVoltage();
    this.stateConsumer = stateConsumer;
    this.periodSec = periodSec;
    this.lastSimTime = Timer.getFPGATimestamp();
    if (useSeparateThread) {
      startSimThread(name);
    }
  }

  /**
   * Sets the voltage supplier used to drive the simulation each step.
   * If {@link TalonFXSubsystemSimConstants#limitVoltage()} is {@code true}, the value is clamped to ±12V.
   *
   * @param simVoltage supplier returning the desired input voltage
   */
  public void setSimVoltage(DoubleSupplier simVoltage) {
    this.simVoltage =
        () ->
            limitVoltage
                ? MathUtil.clamp(simVoltage.getAsDouble(), -12, 12)
                : simVoltage.getAsDouble();
  }

  /** Returns the current sim voltage supplier (already wrapped with optional clamping). */
  public DoubleSupplier getSimVoltage() {
    return simVoltage;
  }

  /**
   * Manually steps the simulation. Must only be called when not using a separate thread;
   * logs a DS error and returns early if the notifier is active.
   */
  public void update() {
    if (useSeparateThread) {
      DriverStation.reportError(
          "Simulation thread: " + name + " | Do not call update if using separate thread!", true);
      return;
    }
    final double currentTime = Timer.getFPGATimestamp();
    deltaTime = currentTime - lastSimTime;
    lastSimTime = currentTime;

    simInterface.setInputVoltage(simVoltage.getAsDouble());
    simInterface.update(deltaTime);
    stateConsumer.accept(simInterface.getPosition(), simInterface.getVelocity());
  }

  /**
   * Starts the background {@link Notifier} thread at the configured period.
   *
   * @param name the notifier thread name
   */
  public void startSimThread(String name) {
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Timer.getFPGATimestamp();
              deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              simInterface.setInputVoltage(simVoltage.getAsDouble());
              simInterface.update(deltaTime);
              stateConsumer.accept(simInterface.getPosition(), simInterface.getVelocity());
            });
    simNotifier.setName(name);
    simNotifier.startPeriodic(periodSec);
  }
}
