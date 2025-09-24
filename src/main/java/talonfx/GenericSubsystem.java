package talonfx;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import config.DeviceConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import talonfx.SubsystemConfiguration.BaseConfig;
import talonfx.SubsystemConfiguration.GenericConfig;
import talonfx.SubsystemConfiguration.SimulationMode;
import talonfx.SubsystemSimulator.GenericSimulator;

public class GenericSubsystem extends SubsystemBase {
  protected final SubsystemController controller;
  protected final SubsystemTelemetry telemetry;
  protected final SubsystemSimulator simulator;

  protected final BaseConfig config;
  protected final TalonFX master;
  protected final TalonFX[] slaves;
  protected final CANcoder cancoder;

  protected final TalonFXConfiguration masterConfig;

  protected final StatusSignal<Angle> masterPositionSignal;
  protected final StatusSignal<AngularVelocity> masterVelocitySignal;

  protected final StatusSignal<Angle> masterRotorPositionSignal;
  protected final StatusSignal<AngularVelocity> masterRotorVelocitySignal;

  protected final double maxUnitsLimit;
  protected final double minUnitsLimit;

  protected final boolean shouldSimulate;

  public GenericSubsystem(GenericConfig config) {
    this.config = config;
    this.telemetry = new SubsystemTelemetry();

    master =
        new TalonFX(config.masterConstants.device().id(), config.masterConstants.device().canbus());
    slaves = new TalonFX[config.slaveConstants.length];

    if (config.canCoderConfig != null) {
      CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
      cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
          config.canCoderConfig.absoluteSensorDiscontinuityPoint;
      cancoderConfig.MagnetSensor.MagnetOffset = config.canCoderConfig.magnetOffset;
      cancoderConfig.MagnetSensor.SensorDirection = config.canCoderConfig.sensorDirection;
      cancoder =
          new CANcoder(config.canCoderConfig.device.id(), config.canCoderConfig.device.canbus());
      cancoder.getConfigurator().apply(cancoderConfig);
    } else {
      cancoder = null;
    }

    masterConfig = new TalonFXConfiguration();

    masterConfig.Feedback.FeedbackRemoteSensorID = config.feedbackRemoteSensorId;
    masterConfig.Feedback.FeedbackRotorOffset = config.feedbackRotorOffset;
    masterConfig.Feedback.FeedbackSensorSource = config.feedbackSensorSource;
    masterConfig.Feedback.RotorToSensorRatio = config.rotorToSensorRatio;
    masterConfig.Feedback.SensorToMechanismRatio = config.sensorToMechRatio;

    maxUnitsLimit = config.maxUnitsLimit - config.softLimitDeadband;
    masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxUnitsLimit;
    masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = config.maxUnitsLimit != 0.0;

    minUnitsLimit = config.minUnitsLimit + config.softLimitDeadband;
    masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minUnitsLimit;
    masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = config.minUnitsLimit != 0.0;

    masterConfig.Slot0 = config.slot0;
    masterConfig.Slot1 = config.slot1;
    masterConfig.Slot2 = config.slot2;

    masterConfig.MotionMagic.MotionMagicCruiseVelocity = config.motionMagicCruiseVelocity;
    masterConfig.MotionMagic.MotionMagicAcceleration = config.motionMagicAcceleration;
    masterConfig.MotionMagic.MotionMagicJerk = config.motionMagicJerk;
    masterConfig.MotionMagic.MotionMagicExpo_kV = config.motionMagicExpo_kV;
    masterConfig.MotionMagic.MotionMagicExpo_kA = config.motionMagicExpo_kA;

    masterConfig.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimit;
    masterConfig.CurrentLimits.SupplyCurrentLowerLimit = config.supplyCurrentLowerLimit;
    masterConfig.CurrentLimits.SupplyCurrentLowerTime = config.supplyCurrentLowerTime;
    masterConfig.CurrentLimits.SupplyCurrentLimitEnable = config.supplyCurrentLimit != 0.0;
    masterConfig.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimit;
    masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    masterConfig.MotorOutput.Inverted = config.masterConstants.invert();
    masterConfig.MotorOutput.NeutralMode = config.neutralMode;

    for (int i = 0; i < slaves.length; i++) {
      slaves[i] =
          new TalonFX(
              config.slaveConstants[i].device().id(), config.slaveConstants[i].device().canbus());

      TalonFX slave = slaves[i];
      TalonFXConfiguration slaveConfig = new TalonFXConfiguration();

      slaveConfig.MotorOutput.Inverted = config.slaveConstants[i].invert();
      slaveConfig.MotorOutput.NeutralMode = config.neutralMode;
      slave.setControl(new StrictFollower(config.masterConstants.device().id()));

      configSlave(slave, slaveConfig);
    }

    configMaster(masterConfig);

    master.getRotorPosition().setUpdateFrequency(25.0);
    master.getRotorVelocity().setUpdateFrequency(25.0);
    master.getPosition().setUpdateFrequency(100.0);
    master.getVelocity().setUpdateFrequency(100.0);

    masterRotorPositionSignal = master.getRotorPosition();
    masterRotorVelocitySignal = master.getRotorVelocity();
    masterPositionSignal = master.getPosition();
    masterVelocitySignal = master.getVelocity();

    shouldSimulate =
        (config.simMode == SimulationMode.AUTO && Utils.isSimulation())
            || config.simMode == SimulationMode.FORCED;

    if (shouldSimulate) {
      simulator = new GenericSimulator(config);
    } else {
      this.simulator = null;
    }

    controller = new SubsystemController(master, slaves, simulator);
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

  public synchronized ControlModeValue getControlMode() {
    return master.getControlMode().getValue();
  }

  public synchronized double getRotorPosition() {
    return shouldSimulate
        ? simulator.getPosition()
        : masterRotorPositionSignal.refresh().getValueAsDouble();
  }

  public synchronized double getPosition() {
    return shouldSimulate
        ? simulator.getPosition() / (config.rotorToSensorRatio * config.sensorToMechRatio)
        : masterPositionSignal.refresh().getValueAsDouble();
  }

  public synchronized double getRotorVelocity() {
    return shouldSimulate
        ? simulator.getPosition()
        : masterRotorVelocitySignal.refresh().getValueAsDouble();
  }

  public synchronized double getVelocity() {
    return shouldSimulate
        ? simulator.getPosition() / (config.rotorToSensorRatio * config.sensorToMechRatio)
        : masterVelocitySignal.refresh().getValueAsDouble();
  }

  public synchronized double getReferencePosition() {
    return shouldSimulate
        ? controller.simController.getGoal().position
        : master.getClosedLoopReference().getValueAsDouble();
  }

  public synchronized double getReferenceVelocity() {
    return shouldSimulate
        ? controller.simController.getGoal().velocity
        : master.getClosedLoopReferenceSlope().getValueAsDouble();
  }

  public synchronized double getAcceleration() {
    return master.getAcceleration().getValueAsDouble();
  }

  public synchronized double getClosedLoopOutput() {
    return shouldSimulate ? controller.simOutput : master.getClosedLoopOutput().getValueAsDouble();
  }
}
