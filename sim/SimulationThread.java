package com.team4522.lib.sim;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team4522.lib.drivers.TalonFXSubsystem;
import com.team4522.lib.drivers.TalonFXSubsystem.ControlType;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.pid.FeedforwardUtil.FFState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc2024.RobotContainer;
import frc2024.constants.Constants;
import frc2024.subsystems.shooter.ShooterConstants;
import lombok.Getter;
import lombok.Setter;

public class SimulationThread {

    private SimInterface simInterface;
    private Notifier simNotifier = null;
    private double lastSimTime;

    private double deltaTime;

    private DoubleSupplier simVoltage = () -> 0.0;

    private Consumer<SimState> stateConsumer;

    private double periodSec;
    
    public SimulationThread(SimWrapper simWrapper, Consumer<SimState> stateConsumer, double periodSec){
        this.simInterface = simWrapper;
        this.stateConsumer = stateConsumer;
        this.periodSec = periodSec;
        startSimThread();
    }

    public void setSimVoltage(DoubleSupplier simVoltage, boolean limitVoltage) {
        this.simVoltage = limitVoltage ? () -> MathUtil.clamp(simVoltage.getAsDouble(), -12, 12) : simVoltage;
    }

    public void setSimVoltage(Function<Double, Double> simVoltage, boolean limitVoltage) {
        this.simVoltage = () -> limitVoltage ? MathUtil.clamp(simVoltage.apply(deltaTime), -12, 12) : simVoltage.apply(deltaTime);
    }

    public void startSimThread(){
        simNotifier = new Notifier(() -> {
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
