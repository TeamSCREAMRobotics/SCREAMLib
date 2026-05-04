package com.teamscreamrobotics.motorcontrol;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import com.teamscreamrobotics.motorcontrol.SmartMotorControllerConfig.FollowerConfig;
import com.teamscreamrobotics.power.PowerConsumer;
import com.teamscreamrobotics.power.PowerConstraint;
import com.teamscreamrobotics.power.PowerManager;
import com.teamscreamrobotics.power.PowerPriority;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
public class TalonFXWrapper implements SmartMotorController, PowerConsumer {

    private final TalonFX motor;
    private final SmartMotorControllerConfig config;
    private final String logPrefix;
    private double horizontalZeroRad = 0.0;

    // Master signals
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Temperature> temperatureSignal;

    // CANcoder
    private final CANcoder cancoder;
    private final StatusSignal<Angle> cancoderPositionSignal;

    // Follower motors and their diagnostic signals
    private final TalonFX[] followerMotors;
    private final StatusSignal<Current>[] followerSupplyCurrentSignals;
    private final StatusSignal<Current>[] followerStatorCurrentSignals;
    private final StatusSignal<Temperature>[] followerTempSignals;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(false);
    private final DynamicMotionMagicVoltage dynMotionMagicRequest = new DynamicMotionMagicVoltage(0, 0, 0).withEnableFOC(false);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private PowerConstraint powerConstraint = PowerConstraint.UNCONSTRAINED;

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

        // Instantiate and configure CANcoder before applyConfiguration() so the
        // feedback sensor source can reference it.
        if (config.cancoder != null) {
            cancoder = config.cancoder.canbus().isEmpty()
                    ? new CANcoder(config.cancoder.canId())
                    : new CANcoder(config.cancoder.canId(), config.cancoder.canbus());
            CANcoderConfiguration cancoderCfg = new CANcoderConfiguration();
            cancoderCfg.MagnetSensor = new MagnetSensorConfigs()
                    .withMagnetOffset(config.cancoder.magnetOffsetRotations());
            cancoder.getConfigurator().apply(cancoderCfg);
        } else {
            cancoder = null;
        }

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

        if (cancoder != null) {
            cancoderPositionSignal = cancoder.getAbsolutePosition();
            cancoderPositionSignal.setUpdateFrequency(50);
            cancoder.optimizeBusUtilization();
        } else {
            cancoderPositionSignal = null;
        }

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

        // Seed TalonFX position from CANcoder absolute reading on boot
        if (cancoderPositionSignal != null && config.cancoder.useAsAbsolutePosition()) {
            cancoderPositionSignal.refresh();
            motor.setPosition(cancoderPositionSignal.getValueAsDouble());
        }

        if (isSimulation) {
            motorSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(motorModel, 0.001, config.gearing),
                    motorModel);
        } else {
            motorSim = null;
        }

        PowerManager.register(this);
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

        FeedbackConfigs feedback = new FeedbackConfigs()
                .withSensorToMechanismRatio(config.gearing);
        if (cancoder != null) {
            feedback.withFeedbackSensorSource(config.usePhoenixPro
                            ? FeedbackSensorSourceValue.FusedCANcoder
                            : FeedbackSensorSourceValue.RemoteCANcoder)
                    .withFeedbackRemoteSensorID(cancoder.getDeviceID());
        }
        cfg.Feedback = feedback;

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

        if (cancoderPositionSignal != null) {
            BaseStatusSignal.refreshAll(cancoderPositionSignal);
            inputs.cancoderPositionRad = cancoderPositionSignal.getValueAsDouble() * 2.0 * Math.PI;
            inputs.cancoderConnected = BaseStatusSignal.isAllGood(cancoderPositionSignal);
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

    private double calculateFeedforward(double velocityRadPerSec, double positionRad) {
        if (config.armFeedforward != null || config.simArmFeedforward != null) {
            ArmFeedforward ff = (isSimulation && config.simArmFeedforward != null)
                    ? config.simArmFeedforward : config.armFeedforward;
            if (ff == null) return 0.0;
            return ff.calculate(positionRad - horizontalZeroRad, velocityRadPerSec);
        }
        if (config.elevatorFeedforward != null || config.simElevatorFeedforward != null) {
            ElevatorFeedforward ff = (isSimulation && config.simElevatorFeedforward != null)
                    ? config.simElevatorFeedforward : config.elevatorFeedforward;
            if (ff == null) return 0.0;
            return ff.calculate(velocityRadPerSec);
        }
        if (config.simpleFeedforward != null || config.simSimpleFeedforward != null) {
            SimpleMotorFeedforward ff = (isSimulation && config.simSimpleFeedforward != null)
                    ? config.simSimpleFeedforward : config.simpleFeedforward;
            if (ff == null) return 0.0;
            return ff.calculate(velocityRadPerSec);
        }
        return 0.0;
    }

    @Override
    public double getHorizontalZeroRad() {
        return horizontalZeroRad;
    }

    @Override
    public void setHorizontalZeroRad(double rad) {
        this.horizontalZeroRad = rad;
    }

    // ── PowerConsumer ──────────────────────────────────────────────────────

    @Override
    public PowerPriority getPowerPriority() {
        return config.powerPriority;
    }

    @Override
    public double estimateDemandWatts() {
        // Use last measured stator current. Before first measurement, assume
        // 30 % of the stator limit as a conservative startup estimate.
        double amps = inputs.statorCurrentAmps > 0.5
                ? inputs.statorCurrentAmps
                : config.statorCurrentLimitAmps * 0.3;
        return amps * 12.0;
    }

    @Override
    public void applyPowerConstraint(PowerConstraint constraint) {
        this.powerConstraint = constraint;
    }

    @Override
    public String getConsumerName() {
        return logPrefix;
    }

    @Override
    public int[] getPDHChannels() {
        return config.pdhChannels;
    }

    // ── Constraint-aware control helpers ──────────────────────────────────

    private void sendPositionControl(double rotations, double ff) {
        if (powerConstraint.fullyDisabled) { motor.stopMotor(); return; }
        if (powerConstraint.isUnconstrained()) {
            motor.setControl(motionMagicRequest
                    .withPosition(rotations)
                    .withFeedForward(ff));
        } else {
            motor.setControl(dynMotionMagicRequest
                    .withPosition(rotations)
                    .withVelocity(config.motionMagic.MotionMagicCruiseVelocity
                            * powerConstraint.velocityCap)
                    .withAcceleration(config.motionMagic.MotionMagicAcceleration
                            * powerConstraint.accelerationCap)
                    .withFeedForward(ff * powerConstraint.feedforwardCap));
        }
    }

    private void sendVelocityControl(double rps, double ff) {
        if (powerConstraint.fullyDisabled) { motor.stopMotor(); return; }
        motor.setControl(velocityRequest
                .withVelocity(rps * powerConstraint.velocityCap)
                .withFeedForward(ff * powerConstraint.feedforwardCap));
    }

    // ── SmartMotorController control methods ──────────────────────────────

    @Override
    public void setPosition(Angle position) {
        double ff = calculateFeedforward(inputs.velocityRadPerSec, inputs.positionRad);
        sendPositionControl(position.in(Rotations), ff);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        double ff = calculateFeedforward(velocity.in(RadiansPerSecond), inputs.positionRad);
        sendVelocityControl(velocity.in(RotationsPerSecond), ff);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        if (powerConstraint.fullyDisabled) { motor.stopMotor(); return; }
        motor.setControl(voltageRequest.withOutput(voltage.in(Volts) * powerConstraint.feedforwardCap));
    }

    @Override
    public void setLinearPosition(Distance position) {
        double rotations = position.in(Meters) / config.mechanismCircumference.in(Meters);
        sendPositionControl(rotations, 0.0);
    }

    @Override
    public void setLinearVelocity(LinearVelocity velocity) {
        double rps = velocity.in(MetersPerSecond) / config.mechanismCircumference.in(Meters);
        sendVelocityControl(rps, 0.0);
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
        if (cancoder != null) {
            var cancoderSimState = cancoder.getSimState();
            cancoderSimState.setRawPosition(position.in(Rotations));
            cancoderSimState.setVelocity(velocity.in(RotationsPerSecond));
        }
    }

    @Override
    public double getSimVoltage() {
        return isSimulation ? motor.getSimState().getMotorVoltage() : 0.0;
    }

    @Override
    public void reconfigure() {
        applyConfiguration();
    }
}
