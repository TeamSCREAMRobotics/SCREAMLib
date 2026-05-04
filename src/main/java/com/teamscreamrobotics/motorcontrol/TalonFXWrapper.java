package com.teamscreamrobotics.motorcontrol;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import com.teamscreamrobotics.motorcontrol.SmartMotorControllerConfig.FollowerConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

/**
 * TalonFX implementation of {@link SmartMotorController}.
 *
 * <p>Follower supply/stator current arrays are logged as replayed inputs
 * specifically to support current-based game piece detection logic that reads
 * follower currents independently. Without logging them as inputs, any replay
 * that depends on follower current thresholds would be nondeterministic because
 * follower signals are not injected from the log during replay.
 */
public class TalonFXWrapper implements SmartMotorController {

    private final TalonFX motor;
    private final SmartMotorControllerConfig config;
    private final String logPrefix;

    // Master signals
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Temperature> temperatureSignal;

    // Follower motors and their diagnostic signals
    private final TalonFX[] followerMotors;
    private final StatusSignal<Current>[] followerSupplyCurrentSignals;
    private final StatusSignal<Current>[] followerStatorCurrentSignals;
    private final StatusSignal<Temperature>[] followerTempSignals;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(false);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final SmartMotorControllerInputsAutoLogged inputs = new SmartMotorControllerInputsAutoLogged();

    private final DCMotorSim motorSim;
    private final boolean isSimulation;

    @SuppressWarnings("unchecked")
    public TalonFXWrapper(TalonFX motor, DCMotor motorModel, SmartMotorControllerConfig config) {
        this.motor = motor;
        this.config = config;
        this.config.motorModel = motorModel;
        this.logPrefix = config.resolveLogPrefix(motor.getDescription());
        this.isSimulation = RobotBase.isSimulation();

        applyConfiguration();

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        statorCurrentSignal = motor.getStatorCurrent();
        temperatureSignal = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(100, positionSignal, velocitySignal);
        BaseStatusSignal.setUpdateFrequencyForAll(50, voltageSignal, supplyCurrentSignal,
                statorCurrentSignal, temperatureSignal);

        motor.optimizeBusUtilization();

        // Build follower motors
        FollowerConfig[] followerConfigs = config.getFollowers();
        int followerCount = followerConfigs.length;

        followerMotors = new TalonFX[followerCount];
        followerSupplyCurrentSignals = new StatusSignal[followerCount];
        followerStatorCurrentSignals = new StatusSignal[followerCount];
        followerTempSignals = new StatusSignal[followerCount];

        for (int i = 0; i < followerCount; i++) {
            FollowerConfig fc = followerConfigs[i];
            TalonFX follower = fc.canbus().isEmpty()
                    ? new TalonFX(fc.canId())
                    : new TalonFX(fc.canId(), fc.canbus());

            // Minimal config: only neutral mode; direction is handled by Follower request.
            TalonFXConfiguration followerCfg = new TalonFXConfiguration();
            followerCfg.MotorOutput = new MotorOutputConfigs()
                    .withNeutralMode(config.idleMode);
            follower.getConfigurator().apply(followerCfg);

            follower.setControl(new Follower(motor.getDeviceID(),
                    fc.oppose() ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
            follower.optimizeBusUtilization();

            followerMotors[i] = follower;
            followerSupplyCurrentSignals[i] = follower.getSupplyCurrent();
            followerStatorCurrentSignals[i] = follower.getStatorCurrent();
            followerTempSignals[i] = follower.getDeviceTemp();

            BaseStatusSignal.setUpdateFrequencyForAll(50,
                    followerSupplyCurrentSignals[i],
                    followerStatorCurrentSignals[i],
                    followerTempSignals[i]);
        }

        // Pre-size follower arrays in inputs to the correct length
        inputs.followerSupplyCurrentAmps = new double[followerCount];
        inputs.followerStatorCurrentAmps = new double[followerCount];
        inputs.followerTempCelsius = new double[followerCount];
        inputs.followerConnected = new boolean[followerCount];

        if (isSimulation) {
            motorSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(motorModel, 0.001, config.gearing),
                    motorModel);
        } else {
            motorSim = null;
        }
    }

    private void applyConfiguration() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput = new MotorOutputConfigs()
                .withInverted(config.inverted ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(config.idleMode);

        cfg.CurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(config.supplyCurrentLimitAmps)
                .withSupplyCurrentLimitEnable(config.enableSupplyCurrentLimit)
                .withStatorCurrentLimit(config.statorCurrentLimitAmps)
                .withStatorCurrentLimitEnable(config.enableStatorCurrentLimit);

        cfg.OpenLoopRamps = new OpenLoopRampsConfigs()
                .withVoltageOpenLoopRampPeriod(config.openLoopRampRate);

        cfg.ClosedLoopRamps = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(config.closedLoopRampRate);

        cfg.Feedback = new FeedbackConfigs()
                .withSensorToMechanismRatio(config.gearing);

        cfg.Slot0 = config.slot0;
        cfg.MotionMagic = config.motionMagic;

        if (config.enableContinuousWrap) {
            cfg.ClosedLoopGeneral = new ClosedLoopGeneralConfigs().withContinuousWrap(true);
        }

        if (config.forwardSoftLimit != null || config.reverseSoftLimit != null) {
            SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
            if (config.forwardSoftLimit != null) {
                softLimits.withForwardSoftLimitThreshold(config.forwardSoftLimit.in(Rotations))
                          .withForwardSoftLimitEnable(true);
            }
            if (config.reverseSoftLimit != null) {
                softLimits.withReverseSoftLimitThreshold(config.reverseSoftLimit.in(Rotations))
                          .withReverseSoftLimitEnable(true);
            }
            cfg.SoftwareLimitSwitch = softLimits;
        }

        motor.getConfigurator().apply(cfg);
    }

    @Override
    public void updateInputs(SmartMotorControllerInputs inputs) {
        // During replay, AKit injects all values via processInputs.
        if (Logger.hasReplaySource()) return;

        BaseStatusSignal.refreshAll(positionSignal, velocitySignal, voltageSignal,
                supplyCurrentSignal, statorCurrentSignal, temperatureSignal);

        inputs.positionRad = BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                positionSignal, velocitySignal) * 2.0 * Math.PI;
        inputs.velocityRadPerSec = velocitySignal.getValueAsDouble() * 2.0 * Math.PI;
        inputs.appliedVolts = voltageSignal.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
        inputs.tempCelsius = temperatureSignal.getValueAsDouble();
        inputs.connected = BaseStatusSignal.isAllGood(positionSignal, velocitySignal,
                voltageSignal, supplyCurrentSignal, statorCurrentSignal, temperatureSignal);

        for (int i = 0; i < followerMotors.length; i++) {
            BaseStatusSignal.refreshAll(
                    followerSupplyCurrentSignals[i],
                    followerStatorCurrentSignals[i],
                    followerTempSignals[i]);

            inputs.followerSupplyCurrentAmps[i] = followerSupplyCurrentSignals[i].getValueAsDouble();
            inputs.followerStatorCurrentAmps[i] = followerStatorCurrentSignals[i].getValueAsDouble();
            inputs.followerTempCelsius[i] = followerTempSignals[i].getValueAsDouble();
            inputs.followerConnected[i] = BaseStatusSignal.isAllGood(
                    followerSupplyCurrentSignals[i],
                    followerStatorCurrentSignals[i],
                    followerTempSignals[i]);
        }
    }

    @Override
    public SmartMotorControllerInputsAutoLogged getInputs() {
        return inputs;
    }

    @Override
    public String getLogPrefix() {
        return logPrefix;
    }

    @Override
    public void setPosition(Angle position) {
        motor.setControl(motionMagicRequest.withPosition(position.in(Rotations)));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(velocityRequest.withVelocity(velocity.in(RotationsPerSecond)));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setControl(voltageRequest.withOutput(voltage.in(Volts)));
    }

    @Override
    public void setLinearPosition(Distance position) {
        double rotations = position.in(Meters) / config.mechanismCircumference.in(Meters);
        motor.setControl(motionMagicRequest.withPosition(rotations));
    }

    @Override
    public void setLinearVelocity(LinearVelocity velocity) {
        double rps = velocity.in(MetersPerSecond) / config.mechanismCircumference.in(Meters);
        motor.setControl(velocityRequest.withVelocity(rps));
    }

    @Override
    public void resetEncoder(Angle position) {
        motor.setPosition(position.in(Rotations));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public Angle getMechanismPosition() {
        return Radians.of(inputs.positionRad);
    }

    @Override
    public AngularVelocity getMechanismVelocity() {
        return RadiansPerSecond.of(inputs.velocityRadPerSec);
    }

    @Override
    public Voltage getVoltage() {
        return Volts.of(inputs.appliedVolts);
    }

    @Override
    public Current getSupplyCurrent() {
        return Amps.of(inputs.supplyCurrentAmps);
    }

    @Override
    public Current getStatorCurrent() {
        return Amps.of(inputs.statorCurrentAmps);
    }

    @Override
    public Temperature getTemperature() {
        return Celsius.of(inputs.tempCelsius);
    }

    @Override
    public SmartMotorControllerConfig getConfig() {
        return config;
    }

    @Override
    public void simIterate(double dtSeconds) {
        if (motorSim == null) return;
        // Physics must not run during replay -- AKit provides all sensor values from the log.
        if (Logger.hasReplaySource()) return;

        // Only the master DCMotorSim is advanced; Phoenix propagates sim state to followers.
        var simState = motor.getSimState();
        motorSim.setInputVoltage(simState.getMotorVoltage());
        motorSim.update(dtSeconds);
        simState.setRawRotorPosition(motorSim.getAngularPositionRotations() * config.gearing);
        simState.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60.0 * config.gearing);
    }

    @Override
    public void simUpdate(Angle position, AngularVelocity velocity) {
        if (!isSimulation || Logger.hasReplaySource()) return;
        var simState = motor.getSimState();
        simState.setRawRotorPosition(position.in(Rotations) * config.gearing);
        simState.setRotorVelocity(velocity.in(RotationsPerSecond) * config.gearing);
    }

    @Override
    public void reconfigure() {
        applyConfiguration();
    }
}
