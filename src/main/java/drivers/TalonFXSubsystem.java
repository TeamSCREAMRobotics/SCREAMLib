package drivers;

import config.DeviceConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import pid.ScreamPIDConstants.MotionMagicConstants;
import sim.SimWrapper;
import sim.SimulationThread;
import java.util.function.DoubleSupplier;
import java.util.function.UnaryOperator;

import javax.swing.text.html.parser.DTD;

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
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

/**
 * Base class for TalonFX based subsystems.
 * Defines subsystems as a set of motors on a mechanism that do the same thing. 
 * This means that each DOF of a mechanism is represented as a separate subsystem.
 * <p> Supports:
 *  <ul>
 *    <li> 1 master TalonFX
 *    <li> Any number of slave TalonFXs
 *    <li> 1 CANCoder (Complex systems with multiple CANCoders should be handled externally)
 *    <li> Simple control requests -- both Motion Magic and non Motion Magic (Velocity, Position, Voltage, Duty Cycle)
 *    <li> User-defined control requests
 *    <li> Multi and single threaded simulation setup through configuration
 *    <li> Logging with DogLog
 */
public class TalonFXSubsystem extends SubsystemBase {

  /**
     * Creates a new CANDevice.
     *
     * @param deviceId ID of the device.
     * @param canbus   Name of the CAN bus this device is on. Possible CAN bus
     *                 strings are:
     *                 <ul>
     *                   <li>"rio" for the native roboRIO CAN bus
     *                   <li>CANivore name or serial number
     *                   <li>SocketCAN interface (non-FRC Linux only)
     *                   <li>"*" for any CANivore seen by the program
     *                   <li>empty string (default) to select the default for the
     *                       system:
     *                   <ul>
     *                     <li>"rio" on roboRIO
     *                     <li>"can0" on Linux
     *                     <li>"*" on Windows
     *                   </ul>
     *                 </ul>
     */
  public static record CANDevice(Integer id, String canbus) {
    public CANDevice() {
      this(null, null);
    }
    public CANDevice(Integer id){
      this(id, "");
    }
  }

  /**
   * Constants for a TalonFX motor controller.
   * 
   * @param device CANDevice constants.
   * @param invert InvertedValue of the device.
   */
  public static record TalonFXConstants(CANDevice device, InvertedValue invert) {
    public TalonFXConstants() {
      this(null, null);
    }
  }

  /**
   * Constants for a CANCoder.
   * 
   * @param device CANDevice constants.
   * @param config Configuration for the device.
   */
  public static record CANCoderConstants(CANDevice device, CANcoderConfiguration config) {}

  public interface TalonFXSubsystemGoal {
    DoubleSupplier target();

    ControlType controlType();

    DoubleSupplier feedForward();
  }

  public record TalonFXSubsystemSimConstants(
      SimWrapper sim,
      double gearing,
      ProfiledPIDController simController,
      boolean useSeparateThread,
      boolean limitVoltage) {
    public TalonFXSubsystemSimConstants(SimWrapper sim, double gearing, PIDController simController) {
      this(sim, gearing, new ProfiledPIDController(simController.getP(), simController.getI(), simController.getD(), new Constraints(9999999, 9999999)), false, false);
    }

    public TalonFXSubsystemSimConstants(SimWrapper sim, double gearing, PIDController simController, double minInput, double maxInput) {
      this(sim, gearing, createContinuousController(simController, minInput, maxInput), false, false);
    }

    private static ProfiledPIDController createContinuousController(PIDController simController, double minInput, double maxInput) {
      ProfiledPIDController controller = new ProfiledPIDController(simController.getP(), simController.getI(), simController.getD(), new Constraints(9999999, 9999999));
      controller.enableContinuousInput(minInput, maxInput);
      return controller;
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

  public static class TalonFXSubsystemConfiguration {
    public String name = "ERROR_ASSIGN_A_NAME";

    public boolean codeEnabled = true;
    public boolean forceSimulation = false;
    public boolean logTelemetry = false;
    public boolean debugMode = false;
    public boolean useCustomSimCallback = false;

    public String logPrefix = null;

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

    public boolean continuousWrap = false;

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

    public double maxUnitsLimit = 0.0;
    public double minUnitsLimit = 0.0;
  }

  protected final TalonFXSubsystemConfiguration config;
  protected final TalonFX master;
  protected final TalonFX[] slaves;
  protected CANcoder cancoder;

  protected TalonFXSubsystemGoal goal;

  protected SimWrapper sim;
  protected SimulationThread simulationThread;
  protected ProfiledPIDController simController;
  protected DoubleSupplier simFeedforwardSup;
  protected double simPosition;
  protected double simVelocity;

  protected TalonFXConfiguration masterConfig;
  protected final TalonFXConfiguration[] slaveConfigs;

  protected final StatusSignal<Angle> masterPositionSignal;
  protected final StatusSignal<AngularVelocity> masterVelocitySignal;

  protected final StatusSignal<Angle> masterRotorPositionSignal;
  protected final StatusSignal<AngularVelocity> masterRotorVelocitySignal;

  protected final double forwardSoftLimitRotations;
  protected final double reverseSoftLimitRotations;

  protected final DutyCycleOut dutyCycleRequest;
  protected final VoltageOut voltageRequest;
  protected final PositionVoltage positionRequest;
  protected final MotionMagicVoltage motionMagicPositionRequest;
  protected final VelocityVoltage velocityRequest;
  protected final MotionMagicVelocityVoltage motionMagicVelocityRequest;

  public final String logPrefix;

  public static final TalonFXSubsystemGoal defaultGoal = new TalonFXSubsystemGoal() {
    @Override
    public DoubleSupplier target() {
      return () -> 0.0;
    }

    @Override
    public ControlType controlType() {
      return ControlType.VOLTAGE;
    }

    public DoubleSupplier feedForward() {
      return () -> 0.0;
    };
  };

  protected double setpoint = 0;
  public boolean inVelocityMode = false;

  protected boolean isEStopped = false;

  public TalonFXSubsystem(
      final TalonFXSubsystemConfiguration config, final TalonFXSubsystemGoal defaultGoal) {
    this.config = config;
    master =
        new TalonFX(config.masterConstants.device.id, config.masterConstants.device.canbus);
    slaves = new TalonFX[config.slaveConstants.length];
    slaveConfigs = new TalonFXConfiguration[config.slaveConstants.length];
    if (config.cancoderConstants != null) {
      cancoder =
          new CANcoder(
              config.cancoderConstants.device.id, config.cancoderConstants.device.canbus);
      CANcoderConfiguration cancoderConfig = config.cancoderConstants.config;
      DeviceConfig.configureCANcoder(config.name + " CANcoder", cancoder, cancoderConfig);
    }

    masterConfig = new TalonFXConfiguration();

    masterConfig.Feedback.FeedbackSensorSource = config.feedbackSensorSource;
    masterConfig.Feedback.FeedbackRemoteSensorID = config.feedbackRemoteSensorId;
    masterConfig.Feedback.FeedbackRotorOffset = config.feedbackRotorOffset;

    masterConfig.ClosedLoopGeneral.ContinuousWrap = config.continuousWrap;

    forwardSoftLimitRotations = (config.maxUnitsLimit - config.softLimitDeadband);
    masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimitRotations;
    masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = config.minUnitsLimit != 0.0;

    reverseSoftLimitRotations = (config.minUnitsLimit + config.softLimitDeadband);
    masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimitRotations;
    masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = config.minUnitsLimit != 0.0;

    masterConfig.Slot0 = config.slot0;
    masterConfig.Slot1 = config.slot1;
    masterConfig.Slot2 = config.slot2;

    masterConfig.MotionMagic.MotionMagicCruiseVelocity = config.cruiseVelocity;
    masterConfig.MotionMagic.MotionMagicAcceleration = config.acceleration;
    masterConfig.MotionMagic.MotionMagicJerk = config.jerk;

    masterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = config.rampRate;
    masterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.rampRate;
    masterConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = config.rampRate;

    masterConfig.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimit;
    masterConfig.CurrentLimits.SupplyCurrentLimitEnable = config.enableSupplyCurrentLimit;
    masterConfig.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimit;
    masterConfig.CurrentLimits.StatorCurrentLimitEnable = config.enableStatorCurrentLimit;

    masterConfig.MotorOutput.Inverted = config.masterConstants.invert;
    masterConfig.Feedback.SensorToMechanismRatio = config.sensorToMechRatio;
    masterConfig.Feedback.RotorToSensorRatio = config.rotorToSensorRatio;
    masterConfig.MotorOutput.NeutralMode = config.neutralMode;

    for (int i = 0; i < slaves.length; ++i) {
      slaves[i] =
          new TalonFX(
              config.slaveConstants[i].device.id, config.slaveConstants[i].device.canbus);

      TalonFX slave = slaves[i];
      TalonFXConfiguration slaveConfig = new TalonFXConfiguration();

      slaveConfig.MotorOutput.Inverted = config.slaveConstants[i].invert;
      slaveConfig.MotorOutput.NeutralMode = config.neutralMode;
      slave.setControl(
          new Follower(
              config.masterConstants.device.id,
              config.slaveConstants[i].invert != config.masterConstants.invert));

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
      sim = config.simConstants.sim();
      simulationThread =
          new SimulationThread(
              config.simConstants,
              this::setSimState,
              config.simPeriodSec,
              config.name + " Sim Thread");
      simController = config.simConstants.simController();
      simFeedforwardSup = () -> 0.0;
    }

    if(defaultGoal != null){
      goal = defaultGoal;
      setDefaultCommand(applyGoalCommand(goal));
    }

    if(config.logPrefix == null){
      config.logPrefix = "Subsystems/" + config.name + "/";
      logPrefix = config.logPrefix;
    } else {
      logPrefix = config.logPrefix;
    }

    if(config.debugMode && shouldSimulate()){
      SmartDashboard.putNumber(config.name + " kP", simController.getP());
      SmartDashboard.putNumber(config.name + " kI", simController.getI());
      SmartDashboard.putNumber(config.name + " kD", simController.getD());
      SmartDashboard.putNumber(config.name + " Velocity", simController.getConstraints().maxVelocity); 
      SmartDashboard.putNumber(config.name + " Acceleration", simController.getConstraints().maxAcceleration);
    }

    System.out.println("[Init] " + config.name + " initialization complete!");
  }

  public TalonFXSubsystem(TalonFXSubsystemConfiguration config){
    this(config, null);
  }

  /**
   * Configures the master motor with the given configuration.
   *
   * @param motorConfig - The config to apply to the master.
   */
  public void configMaster(TalonFXConfiguration motorConfig) {
    DeviceConfig.configureTalonFX(config.name + " Master", master, motorConfig);
  }

  /**
   * Configures a slave motor with the given configuration.
   *
   * @param slave - The slave to apply the config to.
   * @param config - The config to apply to the slave.
   */
  public void configSlave(TalonFX slave, TalonFXConfiguration motorConfig) {
    DeviceConfig.configureTalonFX(config.name + " Slave", slave, motorConfig);
  }

  /**
   * Reconfigures the motors with the given stator current limits.
   *
   * @param currentLimit - Stator current limit to apply.
   * @param enable - Whether the current limit should be enabled.
   */
  public void setStatorCurrentLimit(double currentLimit, boolean enable) {
    changeTalonConfig(
        (conf) -> {
          conf.CurrentLimits.StatorCurrentLimit = currentLimit;
          conf.CurrentLimits.StatorCurrentLimitEnable = enable;
          return conf;
        });
  }

  /**
   * Reconfigures the motors to enable soft limits.
   *
   * @param enable - Whether the soft limits should be enabled.
   */
  public void enableSoftLimits(boolean enable) {
    changeTalonConfig(
        (conf) -> {
          conf.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
          conf.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
          return conf;
        });
  }

  /**
   * Sets the neutral mode of the motors.
   *
   * @param mode - The neutral mode to apply to the motors.
   */
  public void setNeutralMode(NeutralModeValue mode) {
    master.setNeutralMode(mode);
    for (TalonFX slave : slaves) {
      slave.setNeutralMode(mode);
    }
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

  protected boolean shouldSimulate() {
    if (config.forceSimulation && config.simConstants == null) {
      DriverStation.reportError(
          "Could not force simulation in "
              + config.name
              + ", simulation constants were not provided",
          true);
      return false;
    }
    return (Utils.isSimulation() && config.simConstants != null) || config.forceSimulation;
  }

  public synchronized ControlModeValue getControlMode() {
    return master.getControlMode().asSupplier().get();
  }

  public synchronized double getRotorPosition() {
    return shouldSimulate() ? simPosition : masterRotorPositionSignal.asSupplier().get().in(Units.Rotations);
  }

  public synchronized double getPosition() {
    return shouldSimulate() ? simPosition / config.simConstants.gearing  : masterPositionSignal.asSupplier().get().in(Units.Rotations);
  }

  public synchronized double getRotorVelocity() {
    return shouldSimulate() ? simVelocity : masterRotorVelocitySignal.asSupplier().get().in(Units.RotationsPerSecond);
  }

  public synchronized double getVelocity() {
    return shouldSimulate() ? simVelocity / config.simConstants.gearing : masterVelocitySignal.asSupplier().get().in(Units.RotationsPerSecond);
  }

  public synchronized double getSetpoint() {
    return setpoint;
  }

  public synchronized TalonFXSubsystemGoal getGoal() {
    return goal;
  }

  public synchronized double getError() {
    if(goal != null){
      return inVelocityMode
      ? goal.target().getAsDouble() - getVelocity()
      : goal.target().getAsDouble() - getPosition();
    } else {
      return inVelocityMode
      ? setpoint - getVelocity()
      : setpoint - getPosition();
    }
  }

  public synchronized Rotation2d getAngle() {
    return Rotation2d.fromRotations(getPosition());
  }

  public synchronized boolean atGoal() {
    double error =
        inVelocityMode
            ? goal.target().getAsDouble() - getVelocity()
            : goal.target().getAsDouble() - getPosition();
    return inVelocityMode
        ? Math.abs(error) <= config.velocityThreshold
        : Math.abs(error) <= config.positionThreshold;
  }

  public synchronized boolean atGoal(double absTolerance) {
    double error =
        inVelocityMode
            ? goal.target().getAsDouble() - getVelocity()
            : goal.target().getAsDouble() - getPosition();
    return inVelocityMode
        ? Math.abs(error) <= absTolerance
        : Math.abs(error) <= absTolerance;
  }

  public synchronized boolean isActive() {
    return Math.abs(getRotorVelocity()) > 0.0;
  }

  public synchronized Command applyGoalCommand(TalonFXSubsystemGoal goal) {
    return run(() -> applyGoal(goal)).beforeStarting(Commands.runOnce(() -> simController.reset(getPosition(), getVelocity())).onlyIf(() -> shouldSimulate())
    ).withName(config.name + " Apply Goal " + goal.toString());
  }

  public synchronized void applyGoal(TalonFXSubsystemGoal goal) {
    this.goal = goal;
    switch (goal.controlType()) {
      case MOTION_MAGIC_POSITION:
        setSetpointMotionMagicPosition(goal.target().getAsDouble(), goal.feedForward().getAsDouble());
        break;
      case MOTION_MAGIC_VELOCITY:
        setSetpointMotionMagicVelocity(goal.target().getAsDouble(), goal.feedForward().getAsDouble());
        break;
      case POSITION:
        setSetpointPosition(goal.target().getAsDouble(), goal.feedForward().getAsDouble());
        break;
      case VELOCITY:
        setSetpointVelocity(goal.target().getAsDouble(), goal.feedForward().getAsDouble());
        break;
      case VOLTAGE:
        setVoltage(goal.target().getAsDouble(), goal.feedForward().getAsDouble());
        break;
      case DUTY_CYCLE:
        setDutyCycle(goal.target().getAsDouble(), goal.feedForward().getAsDouble());
        break;
      default:
        stop();
        break;
    }
  }

  public synchronized Command applyVoltageCommand(DoubleSupplier voltage) {
    return run(() -> setVoltage(voltage.getAsDouble())).withName("applyVoltage");
  }

  public synchronized Command applyVoltageCommand(
      DoubleSupplier voltage, DoubleSupplier voltageFeedforward) {
    return run(() -> setVoltage(voltage.getAsDouble(), voltageFeedforward.getAsDouble()))
        .withName("applyVoltage");
  }

  public synchronized Command applyControlCommand(ControlRequest control){
    return run(() -> setMaster(control)).withName("applyControl");
  }

  public synchronized void setDutyCycle(double dutyCycle, double dutyCycleFeedforward) {
    setMaster(dutyCycleRequest.withOutput(dutyCycle + dutyCycleFeedforward));
    if(shouldSimulate()){
      simulationThread.setSimVoltage(() -> (dutyCycle * 12) + dutyCycleFeedforward);
    }
  }

  public synchronized void setDutyCycle(double dutyCycle) {
    setDutyCycle(dutyCycle, 0);
  }

  public synchronized void setVoltage(double volts, double voltageFeedForward) {
    setMaster(voltageRequest.withOutput(volts + voltageFeedForward));
    if (shouldSimulate()) {
      simulationThread.setSimVoltage(() -> volts + voltageFeedForward);
    }
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
    setSetpointMotionMagicVelocity(velocity, 0.0);
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
    if (shouldSimulate()) {
      simulationThread.setSimVoltage(() -> simController.calculate(getPosition(), position) + voltageFeedForward);
    }
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
    if (shouldSimulate()) {
      simulationThread.setSimVoltage(() -> simController.calculate(getVelocity(), velocity) + voltageFeedForward);
    }
  }

  public synchronized void setMaster(ControlRequest control) {
    if (config.codeEnabled && !isEStopped) {
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
    if (config.codeEnabled && !isEStopped) {
      simPosition = position;
      simVelocity = velocity;
    }
  }

  protected void setGoal(TalonFXSubsystemGoal goal){
    this.goal = goal;
  }

  public synchronized void resetPosition(double position) {
    master.setPosition(0.0);
  }

  @Override
  public void periodic() {
    if (config.logTelemetry) {
      outputTelemetry();
    }
    if(goal != null){
      DogLog.log(logPrefix + "Goal", goal.toString());
      DogLog.log(logPrefix + "GoalTarget", goal.target().getAsDouble());
    }
    DogLog.log(logPrefix + "Setpoint", setpoint);
    DogLog.log(logPrefix + "Measured", inVelocityMode ? getVelocity() : getPosition());
    if (getCurrentCommand() != null) {
      DogLog.log(logPrefix + "ActiveCommand", getCurrentCommand().getName());
    }
    if(config.debugMode && shouldSimulate()){
      simController.setP(SmartDashboard.getNumber(config.name + " kP", simController.getP()));
      simController.setI(SmartDashboard.getNumber(config.name + " kI", simController.getI()));
      simController.setD(SmartDashboard.getNumber(config.name + " kD", simController.getD()));
      simController.setConstraints(new Constraints(SmartDashboard.getNumber(config.name + " Velocity", simController.getConstraints().maxVelocity), SmartDashboard.getNumber(config.name + " Acceleration", simController.getConstraints().maxAcceleration)));
      DogLog.log(logPrefix + "ControllerPosition", simController.getSetpoint().position);
      DogLog.log(logPrefix + "ControllerVelocity", simController.getSetpoint().velocity);
    }
  }

  @Override
  public void simulationPeriodic() {
    if (config.simConstants != null && !config.simConstants.useSeparateThread() && !config.useCustomSimCallback) {
      simulationThread.update();
    }
  }

  public void outputTelemetry() {
    DogLog.log(
        logPrefix + "AppliedVolts",
        shouldSimulate() ? simulationThread.getSimVoltage().getAsDouble() + simFeedforwardSup.getAsDouble() : voltageRequest.Output);
    DogLog.log(logPrefix + "Position", getPosition());
    DogLog.log(logPrefix + "Velocity", new double[] {getVelocity(), getVelocity() * 60.0});
    DogLog.log(logPrefix + "Rotor Position", getRotorPosition());
    DogLog.log(
        logPrefix + "Rotor Velocity", new double[] {getRotorVelocity(), getRotorVelocity() * 60.0});
    DogLog.log(logPrefix + "Supply Voltage", master.getSupplyVoltage().getValueAsDouble());
    DogLog.log(logPrefix + "Supply Current", master.getSupplyCurrent().getValueAsDouble());
    DogLog.log(logPrefix + "Setpoint", getSetpoint());
    DogLog.log(logPrefix + "Error", getError());
    if(goal != null){
      DogLog.log(logPrefix + "At Goal?", atGoal());
    }
    DogLog.log(logPrefix + "In Velocity Mode", inVelocityMode);
    DogLog.log(logPrefix + "Control Mode", getControlMode().toString());
  }

  public void emergencyStop() {
    stop();
    setNeutralMode(NeutralModeValue.Coast);
    isEStopped = true;
    getCurrentCommand().cancel();
    setVoltage(0);
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
