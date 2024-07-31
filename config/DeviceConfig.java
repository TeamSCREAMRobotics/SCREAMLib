package com.team4522.lib.config;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.ErrorChecker.DeviceConfiguration;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.pid.ScreamPIDConstants.FeedforwardConstants;
import com.team4522.lib.pid.ScreamPIDConstants.MotionMagicConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public class DeviceConfig {

    ////////////////////////////////////// GENERAL CONFIGURATION METHODS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

    public static void configureTalonFX(String name, TalonFX fx, TalonFXConfiguration config, double updateFrequencyHz){
        DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings(){
                return ErrorChecker.hasConfiguredWithoutErrors(
                    fx.getConfigurator().apply(config),
                    fx.getConfigurator().setPosition(0.0),
                    fx.getVelocity().setUpdateFrequency(updateFrequencyHz),
                    fx.getAcceleration().setUpdateFrequency(50.0),
                    fx.getPosition().setUpdateFrequency(updateFrequencyHz),
                    fx.getSupplyVoltage().setUpdateFrequency(10.0),
                    fx.getStatorCurrent().setUpdateFrequency(10.0),
                    fx.getSupplyCurrent().setUpdateFrequency(10.0),
                    fx.getTorqueCurrent().setUpdateFrequency(50.0),
                    fx.getDeviceTemp().setUpdateFrequency(4.0),
                    fx.getControlMode().setUpdateFrequency(4.0));
            }
        };
        ErrorChecker.configureDevice(deviceConfig, name + " " + fx.getDeviceID() + " version " + fx.getVersion(), true);
    }

    public static void configureCANcoder(String name, CANcoder encoder, CANcoderConfiguration config, double updateFrequencyHz){
        DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings(){
                return ErrorChecker.hasConfiguredWithoutErrors(
                    encoder.getConfigurator().apply(config),
                    encoder.getVelocity().setUpdateFrequency(updateFrequencyHz),
                    encoder.getPosition().setUpdateFrequency(updateFrequencyHz),
                    encoder.getSupplyVoltage().setUpdateFrequency(50.0),
                    encoder.getAbsolutePosition().setUpdateFrequency(updateFrequencyHz));
                }
        };
        ErrorChecker.configureDevice(deviceConfig, name + " " + encoder.getDeviceID() + " version " + encoder.getVersion(), true);
    }

    public static void configurePigeon2(String name, Pigeon2 pigeon, Pigeon2Configuration config, double updateFrequencyHz){
        DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings(){
                return ErrorChecker.hasConfiguredWithoutErrors(
                    pigeon.getConfigurator().apply(config),
                    pigeon.setYaw(0),
                    pigeon.getYaw().setUpdateFrequency(updateFrequencyHz),
                    pigeon.getPitch().setUpdateFrequency(updateFrequencyHz),
                    pigeon.getRoll().setUpdateFrequency(updateFrequencyHz),
                    pigeon.getSupplyVoltage().setUpdateFrequency(updateFrequencyHz));
            }
        };
        ErrorChecker.configureDevice(deviceConfig, name + " " + pigeon.getDeviceID() + " version " + pigeon.getVersion(), true);
    }

    public static AudioConfigs FXAudioConfigs(boolean beepOnBoot, boolean beepOnConfig, boolean allowMusicDuringDisabled){
        AudioConfigs config = new AudioConfigs();
        config.BeepOnBoot = beepOnBoot;
        config.BeepOnConfig = beepOnConfig;
        config.AllowMusicDurDisable = allowMusicDuringDisabled;
        return config;
    }

    public static MotorOutputConfigs FXMotorOutputConfig(InvertedValue invert, NeutralModeValue neutralMode){
        MotorOutputConfigs config = new MotorOutputConfigs();
        config.Inverted = invert;
        config.NeutralMode = neutralMode;
        return config;
    }

    public static FeedbackConfigs FXFeedbackConfig(FeedbackSensorSourceValue sensor, int remoteSensorID, double sensorToMechGR, double rotorToSensorGR, Rotation2d sensorOffset){
        FeedbackConfigs config = new FeedbackConfigs();
        config.FeedbackSensorSource = sensor;
        config.FeedbackRemoteSensorID = remoteSensorID;
        config.SensorToMechanismRatio = sensorToMechGR;
        config.RotorToSensorRatio = rotorToSensorGR;
        config.FeedbackRotorOffset = sensorOffset.getRotations();
        return config;
    }

    public static Slot0Configs FXPIDConfig(ScreamPIDConstants constants, FeedforwardConstants ffConstants){
        return constants.getSlot0Configs(ffConstants);
    }

    public static Slot0Configs FXPIDConfig(ScreamPIDConstants constants){
        return constants.getSlot0Configs(new FeedforwardConstants());
    }

    public static OpenLoopRampsConfigs FXOpenLoopRampConfig(double ramp){
        OpenLoopRampsConfigs config = new OpenLoopRampsConfigs();
        config.DutyCycleOpenLoopRampPeriod = ramp;
        return config;
    }

    public static ClosedLoopRampsConfigs FXClosedLoopRampConfig(double ramp){
        ClosedLoopRampsConfigs config = new ClosedLoopRampsConfigs();
        config.DutyCycleClosedLoopRampPeriod = ramp;
        return config;
    }

    public static CurrentLimitsConfigs FXSupplyCurrentLimitsConfig(boolean enable, double limit, double currentThreshold, double timeThreshold){
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.SupplyCurrentLimitEnable = enable;
        config.SupplyCurrentLimit = limit;
        config.SupplyCurrentThreshold = currentThreshold;
        config.SupplyTimeThreshold = timeThreshold;
        return config;
    }

    public static CurrentLimitsConfigs FXCurrentLimitsConfig(boolean supplyEnable, double supplyLimit, double supplyCurrentThreshold, double supplyTimeThreshold, boolean statorEnable, double statorLimit){
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.SupplyCurrentLimitEnable = supplyEnable;
        config.SupplyCurrentLimit = supplyLimit;
        config.SupplyCurrentThreshold = supplyCurrentThreshold;
        config.SupplyTimeThreshold = supplyTimeThreshold;
        config.StatorCurrentLimitEnable = statorEnable;
        config.StatorCurrentLimit = statorLimit;
        return config;
    }

    public static ClosedLoopGeneralConfigs FXClosedLoopGeneralConfig(boolean continuousWrap){
        ClosedLoopGeneralConfigs config = new ClosedLoopGeneralConfigs();
        config.ContinuousWrap = continuousWrap;
        return config;
    }

    public static SoftwareLimitSwitchConfigs FXSoftwareLimitSwitchConfig(boolean enable, double forwardLimit, double reverseLimit){
        SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();
        config.ForwardSoftLimitEnable = enable;
        config.ReverseSoftLimitEnable = enable;
        config.ForwardSoftLimitThreshold = forwardLimit;
        config.ReverseSoftLimitThreshold = reverseLimit;
        return config;
    }

    public static HardwareLimitSwitchConfigs FXHardwareLimitSwitchConfig(boolean forwardEnable, boolean reverseEnable, double forwardPosition, double reversePosition){
        HardwareLimitSwitchConfigs config = new HardwareLimitSwitchConfigs();
        config.ForwardLimitAutosetPositionEnable = forwardEnable;
        config.ReverseLimitAutosetPositionEnable = reverseEnable;
        config.ForwardLimitAutosetPositionValue = forwardPosition;
        config.ReverseLimitAutosetPositionValue = reversePosition;
        return config;
    }

    public static MotionMagicConfigs FXMotionMagicConfig(MotionMagicConstants constants){
        MotionMagicConfigs config = new MotionMagicConfigs();
        config.MotionMagicAcceleration = constants.acceleration();
        config.MotionMagicCruiseVelocity = constants.cruiseVelocity();
        config.MotionMagicJerk = constants.jerk();
        return config;
    }

    public static VoltageConfigs FXVoltageConfig(double peakForwardVoltage, double peakReverseVoltage, double supplyVoltageTimeConstant){
        VoltageConfigs config = new VoltageConfigs();
        config.PeakForwardVoltage = peakForwardVoltage;
        config.PeakReverseVoltage = peakReverseVoltage;
        config.SupplyVoltageTimeConstant = supplyVoltageTimeConstant;
        return config;
    }

    /* public static SlotConfiguration SRXSlotConfig(ScreamPIDConstants constants){
        SlotConfiguration config = new SlotConfiguration();
        config.kP = constants.kP();
        config.kI = constants.kI();
        config.kD = constants.kD();
        config.kF = constants.kF();
        config.integralZone = constants.integralZone();
        config.maxIntegralAccumulator = constants.maxIntegralAccumulator();
        return config;
    } */
}
