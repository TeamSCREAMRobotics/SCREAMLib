package com.SCREAMLib.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class SimulationThread {

  private SimInterface simInterface;
  private Notifier simNotifier = null;
  private double lastSimTime;

  private double deltaTime;

  private DoubleSupplier simVoltage = () -> 0.0;

  private Consumer<SimState> stateConsumer;

  private boolean useSeparateThread;
  private boolean limitVoltage;

  private double periodSec;

  public SimulationThread(
      SimWrapper simWrapper,
      Consumer<SimState> stateConsumer,
      boolean useSeparateThread,
      boolean limitVoltage,
      double periodSec) {
    this.simInterface = simWrapper;
    this.stateConsumer = stateConsumer;
    this.useSeparateThread = useSeparateThread;
    this.limitVoltage = limitVoltage;
    this.periodSec = periodSec;
    if (useSeparateThread) {
      startSimThread();
    }
  }

  public void setSimVoltage(DoubleSupplier simVoltage) {
    this.simVoltage =
        limitVoltage ? () -> MathUtil.clamp(simVoltage.getAsDouble(), -12, 12) : simVoltage;
  }

  public void setSimVoltage(Function<Double, Double> simVoltage) {
    this.simVoltage =
        () ->
            limitVoltage
                ? MathUtil.clamp(simVoltage.apply(deltaTime), -12, 12)
                : simVoltage.apply(deltaTime);
  }

  public void update() {
    if (useSeparateThread) {
      DriverStation.reportError("Do not call update if using separate thread!", true);
      return;
    }
    final double currentTime = Timer.getFPGATimestamp();
    deltaTime = currentTime - lastSimTime;
    lastSimTime = currentTime;

    simInterface.update(deltaTime);
    simInterface.setInputVoltage(simVoltage.getAsDouble());
    stateConsumer.accept(
        new SimState(
            simInterface.getPosition(),
            simInterface.getVelocity(),
            RobotController.getBatteryVoltage()));
  }

  public void startSimThread() {
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Timer.getFPGATimestamp();
              deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              simInterface.update(deltaTime);
              simInterface.setInputVoltage(simVoltage.getAsDouble());
              stateConsumer.accept(
                  new SimState(
                      simInterface.getPosition(),
                      simInterface.getVelocity(),
                      RobotController.getBatteryVoltage()));
            });
    simNotifier.startPeriodic(periodSec);
  }
}
