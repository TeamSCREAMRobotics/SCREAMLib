package com.SCREAMLib.drivers;

import com.SCREAMLib.config.DeviceConfig;
import com.SCREAMLib.pid.ScreamPIDConstants.MotionMagicConstants;
import com.SCREAMLib.sim.SimWrapper;
import com.SCREAMLib.sim.SimulationThread;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.UnaryOperator;

public class TalonFXSubsystem extends SubsystemBase {

  public static record CANDevice(Integer id, String canbus) {
    public CANDevice() {
      this(null, null);
    }
  }

  public static record TalonFXConstants(CANDevice device, InvertedValue invert) {
    public TalonFXConstants() {
      this(null, null);
    }
  }

  public static record CANCoderConstants(CANDevice device, CANcoderConfiguration config) {}

  public interface TalonFXSubsystemGoal {
    DoubleSupplier target();

    ControlType controlType();
  }

  public record TalonFXSubsystemSimConstants(
      SimWrapper sim,
      PIDController simController,
      boolean useSeparateThread,
      boolean limitVoltage) {
    public TalonFXSubsystemSimConstants(SimWrapper sim, PIDController simController) {
      this(sim, simController, false, true);
    }
  }

  public static enum ControlType {
    MOTION_MAGIC_POSITION,
    MOTION_MAGIC_VELOCITY,
    POSITION,
    VELOCITY,
    VOLTAGE,
    DUTY_CYCLE;
  }

  public static class TalonFXSubsystemConstants {
    public String name = "ERROR_ASSIGN_A_NAME";

    public boolean codeEnabled = true;
    public boolean forceSimulation = false;
    public boolean outputTelemetry = false;

    public double loopPeriodSec = 0.02;
    public double simPeriodSec = 0.001;

    public TalonFXSubsystemSimConstants simConstants = null;

    public TalonFXConstants masterConstants = new TalonFXConstants();
    public TalonFXConstants[] slaveConstants = new TalonFXConstants[0];

    public CANCoderConstants cancoderConstants = null;

    public NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public FeedbackSensorSourceValue feedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    public int feedbackRemoteSensorId = 99;
    public double feedbackRotorOffset = 0.0; // rotations
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
  protected CANcoder cancoder;

  protected TalonFXSubsystemGoal goal;

  protected TalonFXSimState masterSimState;
  protected CANcoderSimState cancoderSimState;
  protected SimWrapper sim;
  protected SimulationThread simulationThread;
  protected PIDController simController;
  protected DoubleSupplier simFeedforwardSup;

  protected TalonFXConfiguration masterConfig;
  protected final TalonFXConfiguration[] slaveConfigs;

  protected final StatusSignal<Double> masterPositionSignal;
  protected final StatusSignal<Double> masterVelocitySignal;

  protected final StatusSignal<Double> masterRotorPositionSignal;
  protected final StatusSignal<Double> masterRotorVelocitySignal;

  protected final double forwardSoftLimitRotations;
  protected final double reverseSoftLimitRotations;

  protected final DutyCycleOut dutyCycleRequest;
  protected final VoltageOut voltageRequest;
  protected final PositionVoltage positionRequest;
  protected final MotionMagicVoltage motionMagicPositionRequest;
  protected final VelocityVoltage velocityRequest;
  protected final MotionMagicVelocityVoltage motionMagicVelocityRequest;

  protected double setpoint = 0;
  public boolean inVelocityMode = false;

  protected TalonFXSubsystem(
      final TalonFXSubsystemConstants constants, final TalonFXSubsystemGoal defaultGoal) {
    this.constants = constants;
    master =
        new TalonFX(constants.masterConstants.device.id, constants.masterConstants.device.canbus);
    slaves = new TalonFX[constants.slaveConstants.length];
    slaveConfigs = new TalonFXConfiguration[constants.slaveConstants.length];
    if (constants.cancoderConstants != null) {
      cancoder =
          new CANcoder(
              constants.cancoderConstants.device.id, constants.cancoderConstants.device.canbus);
      CANcoderConfiguration cancoderConfig = constants.cancoderConstants.config;
      DeviceConfig.configureCANcoder(constants.name + " CANcoder", cancoder, cancoderConfig);
    }

    goal = defaultGoal;

    masterConfig = new TalonFXConfiguration();

    masterConfig.Feedback.FeedbackSensorSource = constants.feedbackSensorSource;
    masterConfig.Feedback.FeedbackRemoteSensorID = constants.feedbackRemoteSensorId;
    masterConfig.Feedback.FeedbackRotorOffset = constants.feedbackRotorOffset;

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
      slaves[i] =
          new TalonFX(
              constants.slaveConstants[i].device.id, constants.slaveConstants[i].device.canbus);

      TalonFX slave = slaves[i];
      TalonFXConfiguration slaveConfig = new TalonFXConfiguration();

      slaveConfig.MotorOutput.Inverted = constants.slaveConstants[i].invert;
      slaveConfig.MotorOutput.NeutralMode = constants.neutralMode;
      slave.setControl(
          new Follower(
              constants.masterConstants.device.id,
              constants.slaveConstants[i].invert != constants.masterConstants.invert));

      configSlave(slave, slaveConfig);
    }

    configMaster(masterConfig);

    dutyCycleRequest = new DutyCycleOut(0.0);
    voltageRequest = new VoltageOut(0.0);
    positionRequest = new PositionVoltage(0.0);
    motionMagicPositionRequest = new MotionMagicVoltage(0.0);
    velocityRequest = new VelocityVoltage(0.0);
    motionMagicVelocityRequest = new MotionMagicVelocityVoltage(0.0);

    master.getRotorPosition().setUpdateFrequency(25.0);
    master.getRotorVelocity().setUpdateFrequency(25.0);
    master.getPosition().setUpdateFrequency(100.0);
    master.getVelocity().setUpdateFrequency(100.0);

    masterPositionSignal = master.getPosition();
    masterVelocitySignal = master.getVelocity();
    masterRotorPositionSignal = master.getRotorPosition();
    masterRotorVelocitySignal = master.getRotorVelocity();

    if (shouldSimulate()) {
      masterSimState = master.getSimState();
      if (constants.cancoderConstants != null) {
        cancoderSimState = cancoder.getSimState();
      }
      sim = constants.simConstants.sim();
      simulationThread =
          new SimulationThread(
              constants.simConstants,
              this::setSimState,
              constants.simPeriodSec,
              constants.name + " Sim Thread");
      simController = constants.simConstants.simController();
    }

    setDefaultCommand(applyGoal(goal));
    System.out.println("[Init] " + constants.name + " initialization complete!");
  }

  public void configMaster(TalonFXConfiguration config) {
    DeviceConfig.configureTalonFX(constants.name + " Master", master, config);
  }

  public void configSlave(TalonFX slave, TalonFXConfiguration config) {
    DeviceConfig.configureTalonFX(constants.name + " Slave", slave, config);
  }

  public void setStatorCurrentLimit(double currentLimit, boolean enable) {
    changeTalonConfig(
        (conf) -> {
          conf.CurrentLimits.StatorCurrentLimit = currentLimit;
          conf.CurrentLimits.StatorCurrentLimitEnable = enable;
          return conf;
        });
  }

  public void enableSoftLimits(boolean enable) {
    changeTalonConfig(
        (conf) -> {
          conf.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
          conf.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
          return conf;
        });
  }

  public void setNeutralMode(NeutralModeValue mode) {
    master.setNeutralMode(mode);
    for (TalonFX slave : slaves) {
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

  private boolean shouldSimulate() {
    if (constants.forceSimulation && constants.simConstants == null) {
      DriverStation.reportError(
          "Could not force simulation in "
              + constants.name
              + ", simulation constants were not provided",
          true);
      return false;
    }
    return (Utils.isSimulation() && constants.simConstants != null) || constants.forceSimulation;
  }

  public synchronized ControlModeValue getControlMode() {
    return master.getControlMode().asSupplier().get();
  }

  public synchronized double getRotorPosition() {
    return masterRotorPositionSignal.asSupplier().get();
  }

  public synchronized double getPosition() {
    return masterPositionSignal.asSupplier().get();
  }

  public synchronized double getRotorVelocity() {
    return masterRotorVelocitySignal.asSupplier().get();
  }

  public synchronized double getVelocity() {
    return masterVelocitySignal.asSupplier().get();
  }

  public synchronized TalonFXSimState getSimState() {
    return masterSimState;
  }

  public synchronized double getSetpoint() {
    return setpoint;
  }

  public synchronized TalonFXSubsystemGoal getGoal() {
    return goal;
  }

  public synchronized double getError() {
    return inVelocityMode
        ? goal.target().getAsDouble() - getVelocity()
        : goal.target().getAsDouble() - getPosition();
  }

  public synchronized Rotation2d getAngle() {
    return Rotation2d.fromRotations(getPosition());
  }

  public synchronized double getSimControllerOutput() {
    double output;
    switch (goal.controlType()) {
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
    if (simFeedforwardSup != null) {
      return output + simFeedforwardSup.getAsDouble();
    } else {
      return output;
    }
  }

  public synchronized boolean atGoal() {
    double error =
        inVelocityMode
            ? goal.target().getAsDouble() - getVelocity()
            : goal.target().getAsDouble() - getPosition();
    return inVelocityMode
        ? Math.abs(error) <= constants.velocityThreshold
        : Math.abs(error) <= constants.positionThreshold;
  }

  public synchronized boolean isActive() {
    return Math.abs(masterRotorVelocitySignal.asSupplier().get()) > 0.0;
  }

  public synchronized Command applyGoal(TalonFXSubsystemGoal goal) {
    Command command;
    command =
        run(
            () -> {
              this.goal = goal;
              switch (goal.controlType()) {
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
                case DUTY_CYCLE:
                  setDutyCycle(goal.target().getAsDouble());
                  break;
                default:
                  stop();
                  break;
              }
            });
    if (shouldSimulate()) {
      return command
          .beforeStarting(
              () ->
                  simulationThread.setSimVoltage(
                      goal.controlType() == ControlType.VOLTAGE
                          ? goal.target()
                          : goal.controlType() == ControlType.DUTY_CYCLE
                              ? () -> goal.target().getAsDouble() * 12.0
                              : () -> getSimControllerOutput()))
          .finallyDo(() -> simController.reset())
          .withName(constants.name + ": applyGoal(" + goal.toString() + ")");
    } else {
      return command.withName(constants.name + ": applyGoal(" + goal.toString() + ")");
    }
  }

  public synchronized Command runVoltage(DoubleSupplier voltage) {
    return run(() -> setVoltage(voltage.getAsDouble())).withName(constants.name + ": runVoltage");
  }

  public synchronized void setDutyCycle(double dutyCycle, double dutyCycleFeedforward) {
    setMaster(dutyCycleRequest.withOutput(dutyCycle + dutyCycleFeedforward));
  }

  public synchronized void setDutyCycle(double dutyCycle) {
    setDutyCycle(dutyCycle, 0);
  }

  public synchronized void setVoltage(double volts, double voltageFeedForward) {
    inVelocityMode = false;
    if (shouldSimulate()) {
      simulationThread.setSimVoltage(() -> volts);
    }
    setMaster(voltageRequest.withOutput(volts + voltageFeedForward));
  }

  public synchronized void setVoltage(double volts) {
    setVoltage(volts, 0.0);
  }

  public synchronized void setSetpointPosition(double position, double voltageFeedForward) {
    setTargetPosition(position, voltageFeedForward, false);
  }

  public synchronized void setSetpointPosition(double position) {
    setSetpointPosition(position, 0.0);
  }

  public synchronized void setSetpointMotionMagicPosition(
      double position, double voltageFeedForward) {
    setTargetPosition(position, voltageFeedForward, true);
  }

  public synchronized void setSetpointMotionMagicPosition(double position) {
    setSetpointMotionMagicPosition(position, 0.0);
  }

  public synchronized void setSetpointVelocity(double velocity, double voltageFeedForward) {
    setTargetVelocity(velocity, voltageFeedForward, false);
  }

  public synchronized void setSetpointVelocity(double velocity) {
    setSetpointVelocity(velocity, 0.0);
  }

  public synchronized void setSetpointMotionMagicVelocity(
      double velocity, double voltageFeedForward) {
    setTargetVelocity(velocity, voltageFeedForward, true);
  }

  public synchronized void setSetpointMotionMagicVelocity(double velocity) {
    setSetpointMotionMagicPosition(velocity, 0.0);
  }

  private synchronized void setTargetPosition(
      double position, double voltageFeedForward, boolean motionMagic) {
    setpoint = position;
    inVelocityMode = false;
    ControlRequest control =
        motionMagic
            ? motionMagicPositionRequest.withPosition(position).withFeedForward(voltageFeedForward)
            : positionRequest.withPosition(position).withFeedForward(voltageFeedForward);
    setMaster(control);
  }

  private synchronized void setTargetVelocity(
      double velocity, double voltageFeedForward, boolean motionMagic) {
    setpoint = velocity;
    inVelocityMode = true;
    ControlRequest control =
        motionMagic
            ? motionMagicVelocityRequest.withVelocity(velocity).withFeedForward(voltageFeedForward)
            : velocityRequest.withVelocity(velocity).withFeedForward(voltageFeedForward);
    setMaster(control);
  }

  public synchronized void setMaster(ControlRequest control) {
    if (constants.codeEnabled) {
      master.setControl(control);
    }
  }

  /**
   * Sets the simulation state for the subsystem.
   *
   * <p>The units for each simulation type are as follows:
   *
   * @param position
   *     <ul>
   *       <li>DCMotorSim: Angular Position (Rotations)
   *       <li>ElevatorSim: Linear Position (Meters)
   *       <li>SingleJointedArmSim: Angular Position (Rotations)
   *       <li>FlywheelSim: N/A
   *     </ul>
   *
   * @param velocity
   *     <ul>
   *       <li>DCMotorSim: Angular Velocity (rot/s)
   *       <li>ElevatorSim: Linear Velocity (m/s)
   *       <li>SingleJointedArmSim: Angular Velocity (rot/s)
   *       <li>FlywheelSim: Angular Velocity (rot/s)
   *     </ul>
   */
  public synchronized void setSimState(double position, double velocity) {
    if (constants.codeEnabled) {
      if (constants.cancoderConstants != null) {
        switch (constants.feedbackSensorSource) {
          case FusedCANcoder:
          case RemoteCANcoder:
            cancoderSimState.setRawPosition(
                position / constants.rotorToSensorRatio * constants.sensorToMechRatio);
            break;
          case SyncCANcoder:
          case RotorSensor:
            break;
          default:
            break;
        }
      }
      masterSimState.setRawRotorPosition(position);
      masterSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      masterSimState.setRotorVelocity(velocity);
    }
  }

  public synchronized void resetPosition(double position) {
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
    if (constants.outputTelemetry) {
      outputTelemetry();
    }
    DogLog.log("RobotState/Subsystems/" + constants.name + "/Goal", goal.toString());
    DogLog.log(
        "RobotState/Subsystems/" + constants.name + "/GoalTarget", goal.target().getAsDouble());
    DogLog.log("RobotState/Subsystems/" + constants.name + "/Setpoint", setpoint);
    DogLog.log(
        "RobotState/Subsystems/" + constants.name + "/Actual",
        inVelocityMode ? getVelocity() : getPosition());
    if (getCurrentCommand() != null) {
      DogLog.log(
          "RobotState/Subsystems/" + constants.name + "/ActiveCommand",
          getCurrentCommand().getName());
    }
  }

  @Override
  public void simulationPeriodic() {
    if (constants.simConstants != null && !constants.simConstants.useSeparateThread()) {
      simulationThread.update();
    }
  }

  public void outputTelemetry() {
    DogLog.log(
        "RobotState/Subsystems/" + constants.name + "/AppliedVolts",
        shouldSimulate() ? simulationThread.getSimVoltage().getAsDouble() : voltageRequest.Output);
    DogLog.log("RobotState/Subsystems/" + constants.name + "/Position", getPosition());
    DogLog.log(
        "RobotState/Subsystems/" + constants.name + "/Velocity",
        new double[] {getVelocity(), getVelocity() * 60.0});
    DogLog.log("RobotState/Subsystems/" + constants.name + "/Rotor Position", getRotorPosition());
    DogLog.log(
        "RobotState/Subsystems/" + constants.name + "/Rotor Velocity",
        new double[] {getRotorVelocity(), getRotorVelocity() * 60.0});
    DogLog.log(
        "RobotState/Subsystems/" + constants.name + "/Supply Voltage",
        master.getSupplyVoltage().getValueAsDouble());
    DogLog.log(
        "RobotState/Subsystems/" + constants.name + "/Supply Current",
        master.getSupplyCurrent().getValueAsDouble());
    DogLog.log("RobotState/Subsystems/" + constants.name + "/Setpoint", getSetpoint());
    DogLog.log("RobotState/Subsystems/" + constants.name + "/Error", getError());
    DogLog.log("RobotState/Subsystems/" + constants.name + "/At Goal?", atGoal());
    DogLog.log("RobotState/Subsystems/" + constants.name + "/In Velocity Mode", inVelocityMode);
    DogLog.log(
        "RobotState/Subsystems/" + constants.name + "/Control Mode", getControlMode().toString());
  }

  public void stop() {
    master.stopMotor();
    for (TalonFX slave : slaves) {
      slave.stopMotor();
    }
  }

  public static void stopAll(TalonFXSubsystem... subsystems) {
    for (TalonFXSubsystem sub : subsystems) {
      sub.stop();
    }
  }
}
