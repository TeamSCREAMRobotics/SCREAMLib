package config;

import config.ErrorChecker.DeviceConfiguration;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;
import pid.ScreamPIDConstants.MotionMagicConstants;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class DeviceConfig {

  public static void configureTalonFX(String name, TalonFX fx, TalonFXConfiguration config) {
    DeviceConfiguration deviceConfig =
        new DeviceConfiguration() {
          @Override
          public boolean configureSettings() {
            return ErrorChecker.hasConfiguredWithoutErrors(
                fx.clearStickyFaults(), fx.getConfigurator().apply(config));
          }
        };
    ErrorChecker.configureDevice(
        deviceConfig,
        name
            + " "
            + fx.getDescription()
            + " version "
            + fx.getVersion()
            + "network"
            + fx.getNetwork(),
        true);
  }

  public static void configureCANcoder(
      String name, CANcoder encoder, CANcoderConfiguration config) {
    DeviceConfiguration deviceConfig =
        new DeviceConfiguration() {
          @Override
          public boolean configureSettings() {
            return ErrorChecker.hasConfiguredWithoutErrors(encoder.getConfigurator().apply(config));
          }
        };
    ErrorChecker.configureDevice(
        deviceConfig,
        name
            + " "
            + encoder.getDeviceID()
            + " version "
            + encoder.getVersion()
            + " network "
            + encoder.getNetwork(),
        true);
  }

  public static void configurePigeon2(
      String name, Pigeon2 pigeon, Pigeon2Configuration config, double updateFrequencyHz) {
    DeviceConfiguration deviceConfig =
        new DeviceConfiguration() {
          @Override
          public boolean configureSettings() {
            return ErrorChecker.hasConfiguredWithoutErrors(
                pigeon.getConfigurator().apply(config),
                pigeon.setYaw(0),
                pigeon.getYaw().setUpdateFrequency(updateFrequencyHz),
                pigeon.getPitch().setUpdateFrequency(updateFrequencyHz),
                pigeon.getRoll().setUpdateFrequency(updateFrequencyHz),
                pigeon.getSupplyVoltage().setUpdateFrequency(updateFrequencyHz));
          }
        };
    ErrorChecker.configureDevice(
        deviceConfig, name + " " + pigeon.getDeviceID() + " version " + pigeon.getVersion(), true);
  }

  public static AudioConfigs FXAudioConfigs(
      boolean beepOnBoot, boolean beepOnConfig, boolean allowMusicDuringDisabled) {
    AudioConfigs config = new AudioConfigs();
    config.BeepOnBoot = beepOnBoot;
    config.BeepOnConfig = beepOnConfig;
    config.AllowMusicDurDisable = allowMusicDuringDisabled;
    return config;
  }

  public static MotorOutputConfigs FXMotorOutputConfig(
      InvertedValue invert, NeutralModeValue neutralMode) {
    MotorOutputConfigs config = new MotorOutputConfigs();
    config.Inverted = invert;
    config.NeutralMode = neutralMode;
    return config;
  }

  public static FeedbackConfigs FXFeedbackConfig(
      FeedbackSensorSourceValue sensor,
      int remoteSensorID,
      double sensorToMechGR,
      double rotorToSensorGR,
      Rotation2d sensorOffset) {
    FeedbackConfigs config = new FeedbackConfigs();
    config.FeedbackSensorSource = sensor;
    config.FeedbackRemoteSensorID = remoteSensorID;
    config.SensorToMechanismRatio = sensorToMechGR;
    config.RotorToSensorRatio = rotorToSensorGR;
    config.FeedbackRotorOffset = sensorOffset.getRotations();
    return config;
  }

  public static Slot0Configs FXPIDConfig(
      ScreamPIDConstants constants, FeedforwardConstants ffConstants) {
    return constants.getSlot0Configs(ffConstants);
  }

  public static Slot0Configs FXPIDConfig(ScreamPIDConstants constants) {
    return constants.getSlot0Configs(new FeedforwardConstants());
  }

  public static OpenLoopRampsConfigs FXOpenLoopRampConfig(double ramp) {
    OpenLoopRampsConfigs config = new OpenLoopRampsConfigs();
    config.DutyCycleOpenLoopRampPeriod = ramp;
    return config;
  }

  public static ClosedLoopRampsConfigs FXClosedLoopRampConfig(double ramp) {
    ClosedLoopRampsConfigs config = new ClosedLoopRampsConfigs();
    config.DutyCycleClosedLoopRampPeriod = ramp;
    return config;
  }

  public static CurrentLimitsConfigs FXSupplyCurrentLimitsConfig(
      boolean enable, double lowerLimit, double limit, double limitTime) {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();
    config.SupplyCurrentLimitEnable = enable;
    config.SupplyCurrentLowerLimit = lowerLimit;
    config.SupplyCurrentLimit = limit;
    config.SupplyCurrentLowerTime = limitTime;
    return config;
  }

  public static CurrentLimitsConfigs FXCurrentLimitsConfig(
      boolean supplyEnable,
      double supplyLowerLimit,
      double supplyLimit,
      double supplyLimitTime,
      boolean statorEnable,
      double statorLimit) {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();
    config.SupplyCurrentLimitEnable = supplyEnable;
    config.SupplyCurrentLowerLimit = supplyLowerLimit;
    config.SupplyCurrentLimit = supplyLimit;
    config.SupplyCurrentLowerTime = supplyLimitTime;
    config.StatorCurrentLimitEnable = statorEnable;
    config.StatorCurrentLimit = statorLimit;
    return config;
  }

  public static ClosedLoopGeneralConfigs FXClosedLoopGeneralConfig(boolean continuousWrap) {
    ClosedLoopGeneralConfigs config = new ClosedLoopGeneralConfigs();
    config.ContinuousWrap = continuousWrap;
    return config;
  }

  public static SoftwareLimitSwitchConfigs FXSoftwareLimitSwitchConfig(
      boolean enable, double forwardLimit, double reverseLimit) {
    SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();
    config.ForwardSoftLimitEnable = enable;
    config.ReverseSoftLimitEnable = enable;
    config.ForwardSoftLimitThreshold = forwardLimit;
    config.ReverseSoftLimitThreshold = reverseLimit;
    return config;
  }

  public static HardwareLimitSwitchConfigs FXHardwareLimitSwitchConfig(
      boolean forwardEnable,
      boolean reverseEnable,
      double forwardPosition,
      double reversePosition) {
    HardwareLimitSwitchConfigs config = new HardwareLimitSwitchConfigs();
    config.ForwardLimitAutosetPositionEnable = forwardEnable;
    config.ReverseLimitAutosetPositionEnable = reverseEnable;
    config.ForwardLimitAutosetPositionValue = forwardPosition;
    config.ReverseLimitAutosetPositionValue = reversePosition;
    return config;
  }

  public static MotionMagicConfigs FXMotionMagicConfig(MotionMagicConstants constants) {
    MotionMagicConfigs config = new MotionMagicConfigs();
    config.MotionMagicAcceleration = constants.acceleration();
    config.MotionMagicCruiseVelocity = constants.cruiseVelocity();
    config.MotionMagicJerk = constants.jerk();
    return config;
  }

  public static VoltageConfigs FXVoltageConfig(
      double peakForwardVoltage, double peakReverseVoltage, double supplyVoltageTimeConstant) {
    VoltageConfigs config = new VoltageConfigs();
    config.PeakForwardVoltage = peakForwardVoltage;
    config.PeakReverseVoltage = peakReverseVoltage;
    config.SupplyVoltageTimeConstant = supplyVoltageTimeConstant;
    return config;
  }
}
