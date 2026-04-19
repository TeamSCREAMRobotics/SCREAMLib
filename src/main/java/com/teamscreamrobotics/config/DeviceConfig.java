package com.teamscreamrobotics.config;

import com.teamscreamrobotics.config.ErrorChecker.DeviceConfiguration;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.FeedforwardConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.MotionMagicConstants;
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

/** Factory methods for building CTRE Phoenix 6 configuration objects and configuring devices. */
public class DeviceConfig {

  /**
   * Applies a {@link TalonFXConfiguration} to a TalonFX, retrying until success or timeout.
   *
   * @param name   readable label used in DS error messages
   * @param fx     the TalonFX to configure
   * @param config the configuration to apply
   */
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

  /**
   * Applies a {@link CANcoderConfiguration} to a CANcoder, retrying until success or timeout.
   *
   * @param name    readable label used in DS error messages
   * @param encoder the CANcoder to configure
   * @param config  the configuration to apply
   */
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

  /**
   * Applies a {@link Pigeon2Configuration} and sets yaw/pitch/roll signal update rates.
   *
   * @param name            human-readable label used in DS error messages
   * @param pigeon          the Pigeon2 to configure
   * @param config          the configuration to apply
   * @param updateFrequencyHz signal update frequency in Hz for yaw, pitch, roll, and supply voltage
   */
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

  /**
   * Builds an {@link AudioConfigs} object for a TalonFX.
   *
   * @param beepOnBoot             whether the motor beeps on boot
   * @param beepOnConfig           whether the motor beeps on configuration
   * @param allowMusicDuringDisabled whether Orchestra playback is allowed while disabled
   */
  public static AudioConfigs FXAudioConfigs(
      boolean beepOnBoot, boolean beepOnConfig, boolean allowMusicDuringDisabled) {
    AudioConfigs config = new AudioConfigs();
    config.BeepOnBoot = beepOnBoot;
    config.BeepOnConfig = beepOnConfig;
    config.AllowMusicDurDisable = allowMusicDuringDisabled;
    return config;
  }

  /**
   * Builds a {@link MotorOutputConfigs} for direction and neutral behavior.
   *
   * @param invert      {@link InvertedValue} for motor direction
   * @param neutralMode {@link NeutralModeValue} (brake or coast) when output is zero
   */
  public static MotorOutputConfigs FXMotorOutputConfig(
      InvertedValue invert, NeutralModeValue neutralMode) {
    MotorOutputConfigs config = new MotorOutputConfigs();
    config.Inverted = invert;
    config.NeutralMode = neutralMode;
    return config;
  }

  /**
   * Builds a {@link FeedbackConfigs} for sensor source and gear ratios.
   *
   * @param sensor          the feedback sensor source
   * @param remoteSensorID  CAN ID of the remote sensor (ignored if not using a remote sensor)
   * @param sensorToMechGR  gear ratio from sensor to mechanism output
   * @param rotorToSensorGR gear ratio from rotor to sensor
   * @param sensorOffset    rotational offset applied to the feedback sensor reading
   */
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

  /**
   * Builds {@link Slot0Configs} from PID and feedforward constants.
   *
   * @param constants   PID gains
   * @param ffConstants feedforward constants (kV, kS, kG, kA, gravity type)
   */
  public static Slot0Configs FXPIDConfig(
      ScreamPIDConstants constants, FeedforwardConstants ffConstants) {
    return constants.getSlot0Configs(ffConstants);
  }

  /**
   * Builds {@link Slot0Configs} from PID constants with zeroed feedforward values.
   *
   * @param constants PID gains
   */
  public static Slot0Configs FXPIDConfig(ScreamPIDConstants constants) {
    return constants.getSlot0Configs(new FeedforwardConstants());
  }

  /**
   * Builds an {@link OpenLoopRampsConfigs} with the given duty-cycle ramp period.
   *
   * @param ramp time in seconds from 0 to full output
   */
  public static OpenLoopRampsConfigs FXOpenLoopRampConfig(double ramp) {
    OpenLoopRampsConfigs config = new OpenLoopRampsConfigs();
    config.DutyCycleOpenLoopRampPeriod = ramp;
    return config;
  }

  /**
   * Builds a {@link ClosedLoopRampsConfigs} with the given duty-cycle ramp period.
   *
   * @param ramp time in seconds from 0 to full output
   */
  public static ClosedLoopRampsConfigs FXClosedLoopRampConfig(double ramp) {
    ClosedLoopRampsConfigs config = new ClosedLoopRampsConfigs();
    config.DutyCycleClosedLoopRampPeriod = ramp;
    return config;
  }

  /**
   * Builds a {@link CurrentLimitsConfigs} for supply current limiting only.
   *
   * @param enable     whether supply current limiting is enabled
   * @param lowerLimit supply current lower limit (amps) — the steady-state limit
   * @param limit      peak supply current limit (amps)
   * @param limitTime  time (seconds) supply current may exceed {@code lowerLimit} before throttling
   */
  public static CurrentLimitsConfigs FXSupplyCurrentLimitsConfig(
      boolean enable, double lowerLimit, double limit, double limitTime) {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();
    config.SupplyCurrentLimitEnable = enable;
    config.SupplyCurrentLowerLimit = lowerLimit;
    config.SupplyCurrentLimit = limit;
    config.SupplyCurrentLowerTime = limitTime;
    return config;
  }

  /**
   * Builds a {@link CurrentLimitsConfigs} for both supply and stator current limiting.
   *
   * @param supplyEnable     whether supply current limiting is enabled
   * @param supplyLowerLimit supply current lower (steady-state) limit in amps
   * @param supplyLimit      peak supply current limit in amps
   * @param supplyLimitTime  time (seconds) supply current may exceed {@code supplyLowerLimit}
   * @param statorEnable     whether stator current limiting is enabled
   * @param statorLimit      stator current limit in amps
   */
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

  /**
   * Builds a {@link ClosedLoopGeneralConfigs} with continuous wrap enabled or disabled.
   * Enable for mechanisms that rotate continuously (e.g. swerve steer); disable for limited-range arms.
   *
   * @param continuousWrap whether to enable continuous wrap for the closed-loop setpoint
   */
  public static ClosedLoopGeneralConfigs FXClosedLoopGeneralConfig(boolean continuousWrap) {
    ClosedLoopGeneralConfigs config = new ClosedLoopGeneralConfigs();
    config.ContinuousWrap = continuousWrap;
    return config;
  }

  /**
   * Builds a {@link SoftwareLimitSwitchConfigs} that applies the same enable state to both directions.
   *
   * @param enable       whether both forward and reverse software limits are active
   * @param forwardLimit forward soft limit threshold in mechanism rotations
   * @param reverseLimit reverse soft limit threshold in mechanism rotations
   */
  public static SoftwareLimitSwitchConfigs FXSoftwareLimitSwitchConfig(
      boolean enable, double forwardLimit, double reverseLimit) {
    SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();
    config.ForwardSoftLimitEnable = enable;
    config.ReverseSoftLimitEnable = enable;
    config.ForwardSoftLimitThreshold = forwardLimit;
    config.ReverseSoftLimitThreshold = reverseLimit;
    return config;
  }

  /**
   * Builds a {@link HardwareLimitSwitchConfigs} that auto-sets position on limit switch trigger.
   *
   * @param forwardEnable   whether the forward limit switch auto-sets position
   * @param reverseEnable   whether the reverse limit switch auto-sets position
   * @param forwardPosition position (rotations) set when the forward limit is triggered
   * @param reversePosition position (rotations) set when the reverse limit is triggered
   */
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

  /**
   * Builds a {@link MotionMagicConfigs} from a {@link MotionMagicConstants} record.
   *
   * @param constants cruise velocity, acceleration, and jerk limits
   */
  public static MotionMagicConfigs FXMotionMagicConfig(MotionMagicConstants constants) {
    MotionMagicConfigs config = new MotionMagicConfigs();
    config.MotionMagicAcceleration = constants.acceleration();
    config.MotionMagicCruiseVelocity = constants.cruiseVelocity();
    config.MotionMagicJerk = constants.jerk();
    return config;
  }

  /**
   * Builds a {@link VoltageConfigs} for peak output and supply filtering.
   *
   * @param peakForwardVoltage        maximum forward output voltage
   * @param peakReverseVoltage        maximum reverse output voltage (typically negative)
   * @param supplyVoltageTimeConstant low-pass filter time constant for supply voltage measurement
   */
  public static VoltageConfigs FXVoltageConfig(
      double peakForwardVoltage, double peakReverseVoltage, double supplyVoltageTimeConstant) {
    VoltageConfigs config = new VoltageConfigs();
    config.PeakForwardVoltage = peakForwardVoltage;
    config.PeakReverseVoltage = peakReverseVoltage;
    config.SupplyVoltageTimeConstant = supplyVoltageTimeConstant;
    return config;
  }
}
