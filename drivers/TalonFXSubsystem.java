package com.team4522.lib.drivers;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.UnaryOperator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.pid.ScreamPIDConstants.MotionMagicConstants;
import com.team4522.lib.sim.SimState;
import com.team4522.lib.sim.SimWrapper;
import com.team4522.lib.sim.SimulationThread;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Robot;
import lombok.Getter;
import lombok.Setter;

public class TalonFXSubsystem extends SubsystemBase{

    public static record CanDevice(Integer id, String canbus){
        public CanDevice(){
            this(null, null);
        }
    }

    public static record TalonFXConstants(CanDevice device, InvertedValue invert){
        public TalonFXConstants(){
            this(null, null);
        }
    }

    public static enum ControlType{
        MOTION_MAGIC_POSITION, MOTION_MAGIC_VELOCITY, POSITION, VELOCITY, VOLTAGE, DUTY_CYCLE;
    }

    public static class TalonFXSubsystemConstants {
        public String name = "ERROR_ASSIGN_A_NAME";

        public boolean codeEnabled = true;
        public boolean outputTelemetry = false;

        public double loopPeriodSec = 0.02;
        public double simPeriodSec = 0.001;

        public SimWrapper sim = null;
        public PIDController simController = null;
        public boolean limitSimVoltage = false;

        public TalonFXConstants masterConstants = new TalonFXConstants();
        public TalonFXConstants[] slaveConstants = new TalonFXConstants[0];

        public NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public FeedbackSensorSourceValue feedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        public int feedbackRemoteSensorId = 99;
        public double feedbackRemoteSensorOffset = 0.0; // rotations
        public double rotorToSensorRatio = 1.0;
        public double sensorToMechRatio = 1.0;
        public double softLimitDeadband = 0.0;
        public double velocityThreshold = 0; // rps
        public double positionThreshold = 0; // rotations

        public Slot0Configs slot0 = new Slot0Configs();
        public Slot1Configs slot1 = new Slot1Configs();
        public Slot2Configs slot2 = new Slot2Configs();

        public double velocityFeedforward = 0;
        public double arbitraryFeedforward = 0;
        public double cruiseVelocity = 0; // units/s
        public double acceleration = 0; // units/s^2
        public double jerk = 0; // units/s^3
        public double rampRate = 0.0; // s
        public double maxVoltage = 12.0;

        public int supplyCurrentLimit = 40; // amps
        public boolean enableSupplyCurrentLimit = false;

        public int statorCurrentLimit = 40; // amps
        public boolean enableStatorCurrentLimit = false;

        public double maxUnitsLimit = Double.POSITIVE_INFINITY;
        public double minUnitsLimit = Double.NEGATIVE_INFINITY;
    }
    
    protected final TalonFXSubsystemConstants constants;
    protected final TalonFX master;
    protected final TalonFX[] slaves;

    @Getter
    protected TalonFXSubsystemGoal goal;

    protected TalonFXSimState masterSimState;
    protected SimulationThread simulationThread;
    protected PIDController simController;
    protected DoubleSupplier simFeedforward;

    protected TalonFXConfiguration masterConfig;
    protected final TalonFXConfiguration[] slaveConfigs;

    protected final StatusSignal<Double> masterPositionSignal;
    protected final StatusSignal<Double> masterVelocitySignal;

    protected final double forwardSoftLimitRotations;
    protected final double reverseSoftLimitRotations;

    protected final VoltageOut voltageRequest;
    protected final PositionVoltage positionRequest;
    protected final MotionMagicVoltage motionMagicPositionRequest;
    protected final VelocityVoltage velocityRequest;
    protected final MotionMagicVelocityVoltage motionMagicVelocityRequest;

    protected double setpoint = 0;
    public boolean inVelocityMode = false;

    protected TalonFXSubsystem(final TalonFXSubsystemConstants constants, final TalonFXSubsystemGoal defaultGoal){
        this.constants = constants;
        master = new TalonFX(constants.masterConstants.device.id, constants.masterConstants.device.canbus);
        slaves = new TalonFX[constants.slaveConstants.length];
        slaveConfigs = new TalonFXConfiguration[constants.slaveConstants.length];

        goal = defaultGoal;

        if(Robot.isSimulation() && constants.sim != null){
            masterSimState = master.getSimState();
            simulationThread = new SimulationThread(constants.sim, this::setSimState, constants.simPeriodSec);
            simController = constants.simController;
        }

        voltageRequest = new VoltageOut(0.0);
        positionRequest = new PositionVoltage(0.0);
        motionMagicPositionRequest = new MotionMagicVoltage(0.0);
        velocityRequest = new VelocityVoltage(0.0);
        motionMagicVelocityRequest = new MotionMagicVelocityVoltage(0.0);

        masterPositionSignal = master.getRotorPosition();
        masterVelocitySignal = master.getRotorVelocity();

        masterConfig = new TalonFXConfiguration();

        masterConfig.Feedback.FeedbackSensorSource = constants.feedbackSensorSource;
        masterConfig.Feedback.FeedbackRemoteSensorID = constants.feedbackRemoteSensorId;

        forwardSoftLimitRotations = (constants.maxUnitsLimit - constants.softLimitDeadband);
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimitRotations;
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        reverseSoftLimitRotations = (constants.minUnitsLimit + constants.softLimitDeadband);
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimitRotations;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        masterConfig.Slot0 = constants.slot0;
        masterConfig.Slot1 = constants.slot1;
        masterConfig.Slot2 = constants.slot2;

        masterConfig.MotionMagic.MotionMagicCruiseVelocity = constants.cruiseVelocity;
        masterConfig.MotionMagic.MotionMagicAcceleration = constants.acceleration;
        masterConfig.MotionMagic.MotionMagicJerk = constants.jerk;

        masterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = constants.rampRate;
        masterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = constants.rampRate;
        masterConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = constants.rampRate;

        masterConfig.CurrentLimits.SupplyCurrentLimit = constants.supplyCurrentLimit;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = constants.enableSupplyCurrentLimit;
        masterConfig.CurrentLimits.StatorCurrentLimit = constants.statorCurrentLimit;
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = constants.enableStatorCurrentLimit;

        masterConfig.MotorOutput.Inverted = constants.masterConstants.invert;
        masterConfig.Feedback.SensorToMechanismRatio = constants.sensorToMechRatio;
        masterConfig.Feedback.RotorToSensorRatio = constants.rotorToSensorRatio;
        masterConfig.MotorOutput.NeutralMode = constants.neutralMode;

        for (int i = 0; i < slaves.length; ++i) {
            slaves[i] = new TalonFX(constants.slaveConstants[i].device.id, constants.slaveConstants[i].device.canbus);

            TalonFX slave = slaves[i];
            TalonFXConfiguration slaveConfig = new TalonFXConfiguration();

            slaveConfig.MotorOutput.Inverted = constants.slaveConstants[i].invert;
            slaveConfig.MotorOutput.NeutralMode = constants.neutralMode;
            slave.setControl(new Follower(constants.masterConstants.device.id, constants.slaveConstants[i].invert != constants.masterConstants.invert));

            configSlave(slave, slaveConfig);
        }

        configMaster(masterConfig);
    }

    public void configMaster(TalonFXConfiguration config){
        DeviceConfig.configureTalonFX(constants.name + " Master", master, config, 200);
    }

    public void configSlave(TalonFX slave, TalonFXConfiguration config){
        DeviceConfig.configureTalonFX(constants.name + " Slave", slave, config, 200);
    }

    public void setStatorCurrentLimit(double currentLimit, boolean enable) {
        changeTalonConfig((conf) -> {
            conf.CurrentLimits.StatorCurrentLimit = currentLimit;
            conf.CurrentLimits.StatorCurrentLimitEnable = enable;
            return conf;
        });
    }

    public void enableSoftLimits(boolean enable) {
        changeTalonConfig((conf) -> {
            conf.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
            conf.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
            return conf;
        });
    }

    public void setNeutralMode(NeutralModeValue mode) {
        master.setNeutralMode(mode);
        for(TalonFX slave : slaves){
            slave.setNeutralMode(mode);
        }
    }

    public void changeTalonConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
        for (int i = 0; i < slaves.length; ++i) {
            slaveConfigs[i] = configChanger.apply(slaveConfigs[i]);
        }
        masterConfig = configChanger.apply(masterConfig);
        writeConfigs();
    }

    public void writeConfigs() {
        for (int i = 0; i < slaves.length; ++i) {
            TalonFX slave = slaves[i];
            TalonFXConfiguration slaveConfig = slaveConfigs[i];
            configSlave(slave, slaveConfig);
        }
        configMaster(masterConfig);
    }

    public synchronized ControlModeValue getControlMode(){
        return master.getControlMode().asSupplier().get();
    }

    public synchronized double getRotorPosition(){
        return masterPositionSignal.asSupplier().get();
    }

    public synchronized double getPosition(){
        return masterPositionSignal.asSupplier().get() / constants.rotorToSensorRatio;
    }

    public synchronized double getRotorVelocity(){
        return masterVelocitySignal.asSupplier().get();
    }

    public synchronized double getVelocity(){
        return masterVelocitySignal.asSupplier().get() / constants.rotorToSensorRatio;
    }

    public synchronized TalonFXSimState getSimState(){
        return masterSimState;
    }

    public synchronized double getSetpoint(){
        return setpoint;
    }

    public synchronized double getError(){
        return inVelocityMode ? setpoint - getVelocity() : setpoint - getPosition();
    }

    public synchronized Rotation2d getAngle(){
        return Rotation2d.fromRotations(getPosition());
    }

    public synchronized double getMPS(double wheelCircumference){
        return getVelocity() * wheelCircumference;
    }

    public synchronized double getSimControllerOutput(){
        double output;
        switch(goal.controlType()){
            case POSITION:
            case MOTION_MAGIC_POSITION:
                output = simController.calculate(getPosition(), setpoint);
                break;
            case VELOCITY:
            case MOTION_MAGIC_VELOCITY:
                output = simController.calculate(getVelocity(), setpoint);
                break;
            default:
                output = 0;
                break;
        }
        if(simFeedforward != null){
            return output + simFeedforward.getAsDouble();
        } else{
            return output;
        }
    }

    public synchronized boolean atGoal(){
        double error = inVelocityMode ? goal.target().getAsDouble() - getVelocity() : goal.target().getAsDouble() - getPosition();
        return inVelocityMode ? error <= constants.velocityThreshold : error <= constants.positionThreshold;
    }

    public synchronized boolean isActive(){
        return Math.abs(master.getVelocity().asSupplier().get()) > 0.0;
    }

    public synchronized Command applyGoal(TalonFXSubsystemGoal goal){
        Command command;
        command = run(() -> {
            this.goal = goal;
            switch(goal.controlType()){
                case MOTION_MAGIC_POSITION:
                    setSetpointMotionMagicPosition(goal.target().getAsDouble());
                    break;
                case MOTION_MAGIC_VELOCITY:
                    setSetpointMotionMagicVelocity(goal.target().getAsDouble());
                    break;
                case POSITION:
                    setSetpointPosition(goal.target().getAsDouble());
                    break;
                case VELOCITY:
                    setSetpointVelocity(goal.target().getAsDouble());
                    break;
                case VOLTAGE:
                    setVoltage(goal.target().getAsDouble());
                    break;
                default:
                    stop();
                    break;
            }
        });
        if(Robot.isSimulation() && constants.sim != null){
            return command.beforeStarting(() -> simulationThread.setSimVoltage(goal.controlType() == ControlType.VOLTAGE ? goal.target() : () -> getSimControllerOutput(), constants.limitSimVoltage));
        } else {
            return command;
        }
    }

    public synchronized Command runVoltage(DoubleSupplier voltage){
        if(Robot.isSimulation() && constants.sim != null){
            return run(() -> setVoltage(voltage.getAsDouble())).beforeStarting(() -> simulationThread.setSimVoltage(voltage, constants.limitSimVoltage));
        } else {
            return run(() -> setVoltage(voltage.getAsDouble()));
        }
    }

    public synchronized void setVoltage(double volts, double voltageFeedForward){
        inVelocityMode = false;
        setMaster(voltageRequest.withOutput(volts + voltageFeedForward));
    }

    public synchronized void setVoltage(double volts){
        setVoltage(volts, 0.0);
    }

    public synchronized void setSetpointPosition(double position, double voltageFeedForward){
        setTargetPosition(position, voltageFeedForward, false);
    }

    public synchronized void setSetpointPosition(double position){
        setSetpointPosition(position, 0.0);
    }

    public synchronized void setSetpointMotionMagicPosition(double position, double voltageFeedForward){
        setTargetPosition(position, voltageFeedForward, true);
    }

    public synchronized void setSetpointMotionMagicPosition(double position){
        setSetpointMotionMagicPosition(position, 0.0);
    }

    public synchronized void setSetpointVelocity(double velocity, double voltageFeedForward){
        setTargetVelocity(velocity, voltageFeedForward, false);
    }

    public synchronized void setSetpointVelocity(double velocity){
        setSetpointVelocity(velocity, 0.0);
    }

    public synchronized void setSetpointMotionMagicVelocity(double velocity, double voltageFeedForward){
        setTargetVelocity(velocity, voltageFeedForward, true);
    }

    public synchronized void setSetpointMotionMagicVelocity(double velocity){
        setSetpointMotionMagicPosition(velocity, 0.0);
    }

    private synchronized void setTargetPosition(double position, double voltageFeedForward, boolean motionMagic){
        setpoint = position;
        inVelocityMode = false;
        ControlRequest control = 
            motionMagic 
                ? motionMagicPositionRequest.withPosition(position).withFeedForward(voltageFeedForward)
                : positionRequest.withPosition(position).withFeedForward(voltageFeedForward);
        setMaster(control);
    }

    private synchronized void setTargetVelocity(double velocity, double voltageFeedForward, boolean motionMagic){
        setpoint = velocity;
        inVelocityMode = true;
        ControlRequest control = 
            motionMagic 
                ? motionMagicVelocityRequest.withVelocity(velocity).withFeedForward(voltageFeedForward)
                : velocityRequest.withVelocity(velocity).withFeedForward(voltageFeedForward);
        setMaster(control);
    }

    public synchronized void setMaster(ControlRequest control){
        if(constants.codeEnabled){
            master.setControl(control);
        }
    }

    public synchronized void setSimState(SimState simState){;
        if(constants.codeEnabled){
            masterSimState.setRawRotorPosition(simState.position());
            masterSimState.setSupplyVoltage(simState.supplyVoltage());
            masterSimState.setRotorVelocity(simState.velocity());
        }
    }

    public synchronized void resetPosition(double position){
        master.setPosition(0.0);
    }

    public synchronized void setSupplyCurrentLimit(double value, boolean enable) {
        masterConfig.CurrentLimits.SupplyCurrentLimit = value;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

        configMaster(masterConfig);
    }

    public synchronized void setSupplyCurrentLimitUnchecked(double value, boolean enable) {
        masterConfig.CurrentLimits.SupplyCurrentLimit = value;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

        master.getConfigurator().apply(masterConfig);
    }

    public synchronized void setStatorCurrentLimitUnchecked(double value, boolean enable) {
        masterConfig.CurrentLimits.StatorCurrentLimit = value;
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = enable;

        master.getConfigurator().apply(masterConfig);
    }

    public synchronized void setMotionMagicConfigsUnchecked(MotionMagicConstants configs) {
        masterConfig.MotionMagic.MotionMagicAcceleration = configs.acceleration();
        masterConfig.MotionMagic.MotionMagicJerk = configs.jerk();
        masterConfig.MotionMagic.MotionMagicCruiseVelocity = configs.cruiseVelocity();

        master.getConfigurator().apply(masterConfig.MotionMagic);
    }

    public synchronized void setMotionMagicConfigs(MotionMagicConstants configs) {
        masterConfig.MotionMagic.MotionMagicAcceleration = configs.acceleration();
        masterConfig.MotionMagic.MotionMagicJerk = configs.jerk();
        masterConfig.MotionMagic.MotionMagicCruiseVelocity = configs.cruiseVelocity();
        
        configMaster(masterConfig);
    }

    @Override
    public void periodic() {
        if(constants.outputTelemetry){
            outputTelemetry();
        }
        Logger.recordOutput("RobotState/Subsystems/" + constants.name + "/Goal", goal.toString());
    }

    public void outputTelemetry(){
        Logger.recordOutput("RobotState/Subsystems/" + constants.name + "/Position", getPosition());
        Logger.recordOutput("RobotState/Subsystems/" + constants.name + "/Velocity", new double[]{getVelocity(), getVelocity() * 60.0});
        Logger.recordOutput("RobotState/Subsystems/" + constants.name + "/Rotor Position", getRotorPosition());
        Logger.recordOutput("RobotState/Subsystems/" + constants.name + "/Rotor Velocity", new double[]{getRotorVelocity(), getRotorVelocity() * 60.0});
        Logger.recordOutput("RobotState/Subsystems/" + constants.name + "/Supply Voltage", master.getSupplyVoltage().getValueAsDouble());
        Logger.recordOutput("RobotState/Subsystems/" + constants.name + "/Supply Current", master.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("RobotState/Subsystems/" + constants.name + "/Setpoint", getSetpoint());
        Logger.recordOutput("RobotState/Subsystems/" + constants.name + "/Error", getError());
        Logger.recordOutput("RobotState/Subsystems/" + constants.name + "/At Goal?", atGoal());
        Logger.recordOutput("RobotState/Subsystems/" + constants.name + "/In Velocity Mode", inVelocityMode);
        Logger.recordOutput("RobotState/Subsystems/" + constants.name + "/Control Mode", getControlMode().toString());
    }

    public void stop() {
        master.stopMotor();
        for(TalonFX slave : slaves){
            slave.stopMotor();
        }
    }
}