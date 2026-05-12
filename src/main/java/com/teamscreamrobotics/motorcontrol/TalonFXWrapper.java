package com.teamscreamrobotics.motorcontrol;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.teamscreamrobotics.power.PowerConsumer;
import com.teamscreamrobotics.power.PowerConstraint;
import com.teamscreamrobotics.power.PowerManager;
import com.teamscreamrobotics.power.PowerPriority;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

/**
 * Concrete TalonFX motor controller wrapper used by all SCREAMLib mechanisms.
 *
 * <p>Construct this wrapper, pass it to a mechanism config (e.g. {@link ArmConfig}), and let
 * the mechanism config apply hardware settings via {@link #applyMechanismConfig}.
 * Direct interaction with this class beyond construction is not normally required.
 *
 * <p>Follower supply/stator current arrays are logged as replayed inputs specifically to support
 * current-based game piece detection that reads follower currents during replay.
 */
public class TalonFXWrapper implements PowerConsumer {

    /**
     * Lightweight runtime info returned by {@link #getRuntimeInfo()}.
     * Read-only; not user-facing.
     *
     * @param gearing                Overall sensor-to-mechanism gear reduction applied by the TalonFX.
     * @param mechanismCircumference Drum or sprocket circumference for linear conversion; null for rotational.
     * @param subsystem              Owning subsystem, used as the command requirement.
     */
    public record RuntimeMotorInfo(
            double gearing,
            Distance mechanismCircumference,
            SubsystemBase subsystem) {}

    private final TalonFX motor;
    final SubsystemBase subsystem;
    private final boolean isSimulation;

    // Master motor signals — initialized in constructor, never reassigned.
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

    // CANcoder — lazily initialized in applyMechanismConfig
    private CANcoder cancoder;
    private StatusSignal<Angle> cancoderPositionSignal;

    // Hardware configuration — set by applyMechanismConfig
    private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    private TalonFXConfiguration simTalonFXConfig = null;
    private Distance mechanismCircumference = null;
    private ArmFeedforward armFeedforward = null;
    private ArmFeedforward simArmFeedforward = null;
    private ElevatorFeedforward elevatorFeedforward = null;
    private ElevatorFeedforward simElevatorFeedforward = null;
    private SimpleMotorFeedforward simpleFeedforward = null;
    private SimpleMotorFeedforward simSimpleFeedforward = null;
    private double horizontalZeroRad = 0.0;
    private PowerPriority powerPriority = PowerPriority.MEDIUM;
    private int[] pdhChannels = new int[0];
    private String logPrefix;

    private PowerConstraint powerConstraint = PowerConstraint.UNCONSTRAINED;
    private double feedforwardOverrideVolts = Double.NaN;

    // Sim-only: manual PID+profile since Phoenix 6 closed-loop doesn't run in WPILib sim.
    // ProfiledPIDController handles position control; simVelocityKp covers velocity control.
    private ProfiledPIDController simPositionPID = null;
    private double simVelocityKp = 0.0;

    private final TalonFXInputsAutoLogged inputs = new TalonFXInputsAutoLogged();

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(false);
    private final DynamicMotionMagicVoltage dynMotionMagicRequest =
            new DynamicMotionMagicVoltage(0, 0, 0).withEnableFOC(false);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0).withEnableFOC(false);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    // ── Constructors ──────────────────────────────────────────────────────────

    /** Convenience overload — no followers. */
    public TalonFXWrapper(TalonFX motor, DCMotor motorModel, SubsystemBase subsystem) {
        this(motor, motorModel, subsystem, new FollowerConfig[0]);
    }

    @SuppressWarnings("unchecked")
    public TalonFXWrapper(TalonFX motor, DCMotor motorModel, SubsystemBase subsystem,
                          FollowerConfig... followers) {
        this.motor = motor;
        this.subsystem = subsystem;
        this.isSimulation = RobotBase.isSimulation();
        this.logPrefix = "Motors/" + subsystem.getClass().getSimpleName()
                + "/" + motor.getDescription() + "/";

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

        followerMotors = new TalonFX[followers.length];
        followerSupplyCurrentSignals = new StatusSignal[followers.length];
        followerStatorCurrentSignals = new StatusSignal[followers.length];
        followerTempSignals = new StatusSignal[followers.length];

        for (int i = 0; i < followers.length; i++) {
            FollowerConfig fc = followers[i];
            TalonFX follower = fc.canbus().isEmpty()
                    ? new TalonFX(fc.canId())
                    : new TalonFX(fc.canId(), fc.canbus());
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

        inputs.followerSupplyCurrentAmps = new double[followers.length];
        inputs.followerStatorCurrentAmps = new double[followers.length];
        inputs.followerTempCelsius = new double[followers.length];
        inputs.followerConnected = new boolean[followers.length];

        PowerManager.register(this);
    }

    // ── Mechanism config application ──────────────────────────────────────────

    /**
     * Applies hardware configuration from a mechanism config. Called once by the mechanism
     * constructor; not user-facing.
     */
    void applyMechanismConfig(MechanismApplyConfig cfg) {
        this.logPrefix = cfg.logPrefix();
        this.talonFXConfig = cfg.talonFXConfig();
        this.simTalonFXConfig = cfg.simTalonFXConfig();
        this.mechanismCircumference = cfg.mechanismCircumference();
        this.armFeedforward = cfg.armFeedforward();
        this.simArmFeedforward = cfg.simArmFeedforward();
        this.elevatorFeedforward = cfg.elevatorFeedforward();
        this.simElevatorFeedforward = cfg.simElevatorFeedforward();
        this.simpleFeedforward = cfg.simpleFeedforward();
        this.simSimpleFeedforward = cfg.simSimpleFeedforward();
        this.horizontalZeroRad = cfg.horizontalZeroRad();
        this.powerPriority = cfg.powerPriority();
        this.pdhChannels = cfg.pdhChannels();

        if (cfg.cancoderConfig() != null && cancoder == null) {
            cancoder = cfg.cancoderCanbus() == null || cfg.cancoderCanbus().isEmpty()
                    ? new CANcoder(cfg.cancoderCanId())
                    : new CANcoder(cfg.cancoderCanId(), cfg.cancoderCanbus());
            cancoder.getConfigurator().apply(cfg.cancoderConfig());
            cancoderPositionSignal = cancoder.getAbsolutePosition();
            cancoderPositionSignal.setUpdateFrequency(50);
            cancoder.optimizeBusUtilization();
            if (cfg.seedFromCANcoder()) {
                cancoderPositionSignal.refresh();
                motor.setPosition(cancoderPositionSignal.getValueAsDouble());
            }
        }

        applyConfiguration();
        if (isSimulation) rebuildSimPID();
    }

    private void rebuildSimPID() {
        TalonFXConfiguration activeCfg = simTalonFXConfig != null ? simTalonFXConfig : talonFXConfig;
        double kP = activeCfg.Slot0.kP;
        double kD = activeCfg.Slot0.kD;
        double maxVel = activeCfg.MotionMagic.MotionMagicCruiseVelocity;
        double maxAccel = activeCfg.MotionMagic.MotionMagicAcceleration;
        simVelocityKp = kP;
        TrapezoidProfile.Constraints constraints = (maxVel > 0 && maxAccel > 0)
                ? new TrapezoidProfile.Constraints(maxVel, maxAccel)
                : new TrapezoidProfile.Constraints(Double.MAX_VALUE, Double.MAX_VALUE);
        if (simPositionPID == null) {
            simPositionPID = new ProfiledPIDController(kP, 0, kD, constraints);
        } else {
            simPositionPID.setPID(kP, 0, kD);
            simPositionPID.setConstraints(constraints);
        }
    }

    private void applyConfiguration() {
        TalonFXConfiguration cfg = (isSimulation && simTalonFXConfig != null)
                ? simTalonFXConfig : talonFXConfig;
        motor.getConfigurator().apply(cfg);

        // Keep follower neutral modes in sync with master
        for (TalonFX follower : followerMotors) {
            TalonFXConfiguration fc = new TalonFXConfiguration();
            fc.MotorOutput = new MotorOutputConfigs()
                    .withNeutralMode(cfg.MotorOutput.NeutralMode);
            follower.getConfigurator().apply(fc);
        }
    }

    // ── Runtime info ──────────────────────────────────────────────────────────

    public RuntimeMotorInfo getRuntimeInfo() {
        return new RuntimeMotorInfo(
                talonFXConfig.Feedback.SensorToMechanismRatio,
                mechanismCircumference,
                subsystem);
    }

    // ── Gain updates (used by MechanismTuner) ─────────────────────────────────

    public void setClosedLoopGains(double kP, double kI, double kD) {
        talonFXConfig.Slot0.kP = kP;
        talonFXConfig.Slot0.kI = kI;
        talonFXConfig.Slot0.kD = kD;
        applyConfiguration();
        if (isSimulation) rebuildSimPID();
    }

    public void setMotionMagicConstraints(double cruiseVelocityRps, double accelerationRps2) {
        talonFXConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocityRps;
        talonFXConfig.MotionMagic.MotionMagicAcceleration = accelerationRps2;
        applyConfiguration();
        if (isSimulation) rebuildSimPID();
    }

    // ── Control methods ───────────────────────────────────────────────────────

    public void setPositionProfiled(Rotation2d position) {
        if (isSimulation) { sendSimPositionVoltage(position.getRotations(), true); return; }
        double ff = calculateFeedforward(inputs.velocityRadPerSec, inputs.positionRad);
        sendPositionControl(position.getRotations(), ff);
    }

    public void setPosition(Rotation2d position) {
        if (powerConstraint.fullyDisabled) { motor.stopMotor(); return; }
        if (isSimulation) { sendSimPositionVoltage(position.getRotations(), false); return; }
        double ff = calculateFeedforward(inputs.velocityRadPerSec, inputs.positionRad);
        motor.setControl(positionVoltageRequest
                .withPosition(position.getRotations())
                .withFeedForward(ff * powerConstraint.feedforwardCap));
    }

    public void setVelocity(AngularVelocity velocity) {
        if (isSimulation) { sendSimVelocityVoltage(velocity.in(RotationsPerSecond)); return; }
        double ff = calculateFeedforward(velocity.in(RadiansPerSecond), inputs.positionRad);
        sendVelocityControl(velocity.in(RotationsPerSecond), ff);
    }

    public void setVoltage(Voltage voltage) {
        if (powerConstraint.fullyDisabled) { motor.stopMotor(); return; }
        motor.setControl(voltageRequest.withOutput(voltage.in(Volts) * powerConstraint.feedforwardCap));
    }

    public void setLinearPositionProfiled(Distance position) {
        if (mechanismCircumference == null) return;
        double rotations = position.in(Meters) / mechanismCircumference.in(Meters);
        if (isSimulation) { sendSimPositionVoltage(rotations, true); return; }
        double ff = calculateFeedforward(inputs.velocityRadPerSec, inputs.positionRad);
        sendPositionControl(rotations, ff);
    }

    public void setLinearPosition(Distance position) {
        if (powerConstraint.fullyDisabled || mechanismCircumference == null) { motor.stopMotor(); return; }
        double rotations = position.in(Meters) / mechanismCircumference.in(Meters);
        if (isSimulation) { sendSimPositionVoltage(rotations, false); return; }
        double ff = calculateFeedforward(inputs.velocityRadPerSec, inputs.positionRad);
        motor.setControl(positionVoltageRequest
                .withPosition(rotations)
                .withFeedForward(ff * powerConstraint.feedforwardCap));
    }

    public void setLinearVelocity(LinearVelocity velocity) {
        if (mechanismCircumference == null) return;
        double rps = velocity.in(MetersPerSecond) / mechanismCircumference.in(Meters);
        if (isSimulation) { sendSimVelocityVoltage(rps); return; }
        double velRadPerSec = rps * 2.0 * Math.PI;
        double ff = calculateFeedforward(velRadPerSec, inputs.positionRad);
        sendVelocityControl(rps, ff);
    }

    public void resetEncoder(Rotation2d position) {
        motor.setPosition(position.getRotations());
        if (isSimulation && simPositionPID != null) {
            simPositionPID.reset(position.getRotations());
        }
    }

    public void stop() {
        motor.stopMotor();
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    public Rotation2d getMechanismPosition() { return new Rotation2d(inputs.positionRad); }
    public AngularVelocity getMechanismVelocity() { return RadiansPerSecond.of(inputs.velocityRadPerSec); }
    public Voltage getVoltage() { return Volts.of(inputs.appliedVolts); }
    public Current getSupplyCurrent() { return Amps.of(inputs.supplyCurrentAmps); }
    public Current getStatorCurrent() { return Amps.of(inputs.statorCurrentAmps); }
    public Temperature getTemperature() { return Celsius.of(inputs.tempCelsius); }

    public Distance getLinearPosition() {
        if (mechanismCircumference == null) return Meters.of(0.0);
        return Meters.of(getMechanismPosition().getRadians() / (2.0 * Math.PI) * mechanismCircumference.in(Meters));
    }

    public LinearVelocity getLinearVelocity() {
        if (mechanismCircumference == null) return MetersPerSecond.of(0.0);
        return MetersPerSecond.of(getMechanismVelocity().in(RadiansPerSecond) / (2.0 * Math.PI) * mechanismCircumference.in(Meters));
    }

    public Rotation2d getCANcoderPosition() {
        return new Rotation2d(inputs.cancoderPositionRad);
    }

    public TalonFXInputsAutoLogged getInputs() { return inputs; }
    public String getLogPrefix() { return logPrefix; }

    public double getHorizontalZeroRad() { return horizontalZeroRad; }
    public void setHorizontalZeroRad(double rad) { this.horizontalZeroRad = rad; }

    public void setFeedforwardOverride(double volts) {
        this.feedforwardOverrideVolts = volts;
    }

    // ── Inputs / sim ──────────────────────────────────────────────────────────

    public void updateInputs(TalonFXInputs inputs) {
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
            BaseStatusSignal.refreshAll(followerSupplyCurrentSignals[i],
                    followerStatorCurrentSignals[i], followerTempSignals[i]);
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

    public void simIterate(double dtSeconds) {}

    public void simUpdate(Rotation2d position, AngularVelocity velocity) {
        if (!isSimulation || Logger.hasReplaySource()) return;
        var simState = motor.getSimState();
        double gearing = talonFXConfig.Feedback.SensorToMechanismRatio;
        simState.setRawRotorPosition(position.getRotations() * gearing);
        simState.setRotorVelocity(velocity.in(RotationsPerSecond) * gearing);
        if (cancoder != null) {
            var cs = cancoder.getSimState();
            cs.setRawPosition(position.getRotations());
            cs.setVelocity(velocity.in(RotationsPerSecond));
        }
    }

    public double getSimVoltage() {
        return isSimulation ? motor.getSimState().getMotorVoltage() : 0.0;
    }

    public void reconfigure() {
        applyConfiguration();
    }

    // ── PowerConsumer ──────────────────────────────────────────────────────────

    @Override public PowerPriority getPowerPriority() { return powerPriority; }

    @Override
    public double estimateDemandWatts() {
        double amps = inputs.statorCurrentAmps > 0.5
                ? inputs.statorCurrentAmps
                : talonFXConfig.CurrentLimits.StatorCurrentLimit * 0.3;
        return amps * 12.0;
    }

    @Override
    public void applyPowerConstraint(PowerConstraint constraint) {
        this.powerConstraint = constraint;
    }

    @Override public String getConsumerName() { return logPrefix; }
    @Override public int[] getPDHChannels() { return pdhChannels; }

    // ── Private helpers ───────────────────────────────────────────────────────

    private void sendPositionControl(double rotations, double ff) {
        if (powerConstraint.fullyDisabled) { motor.stopMotor(); return; }
        if (powerConstraint.isUnconstrained()) {
            motor.setControl(motionMagicRequest.withPosition(rotations).withFeedForward(ff));
        } else {
            motor.setControl(dynMotionMagicRequest
                    .withPosition(rotations)
                    .withVelocity(talonFXConfig.MotionMagic.MotionMagicCruiseVelocity
                            * powerConstraint.velocityCap)
                    .withAcceleration(talonFXConfig.MotionMagic.MotionMagicAcceleration
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

    private void sendSimPositionVoltage(double setpointRotations, boolean profiled) {
        if (powerConstraint.fullyDisabled) { motor.stopMotor(); return; }
        double posRotations = inputs.positionRad / (2.0 * Math.PI);

        double pidOutput;
        double profileVelRps;
        if (profiled && simPositionPID != null) {
            simPositionPID.setGoal(setpointRotations);
            pidOutput = simPositionPID.calculate(posRotations);
            profileVelRps = simPositionPID.getSetpoint().velocity;
        } else {
            pidOutput = simPositionPID != null ? simPositionPID.calculate(posRotations, setpointRotations) : 0.0;
            profileVelRps = 0.0;
        }

        double ff = calculateFeedforward(profileVelRps * 2.0 * Math.PI, inputs.positionRad);
        double voltage = Math.max(-12.0, Math.min(12.0, pidOutput + ff));
        motor.setControl(voltageRequest.withOutput(voltage * powerConstraint.feedforwardCap));

        Logger.recordOutput(logPrefix + "Sim/PositionSetpointRotations", setpointRotations);
        Logger.recordOutput(logPrefix + "Sim/ProfileSetpointRotations",
                profiled && simPositionPID != null ? simPositionPID.getSetpoint().position : setpointRotations);
        Logger.recordOutput(logPrefix + "Sim/VoltageOutput", voltage);
    }

    private void sendSimVelocityVoltage(double setpointRps) {
        if (powerConstraint.fullyDisabled) { motor.stopMotor(); return; }
        double velRps = inputs.velocityRadPerSec / (2.0 * Math.PI);
        double ff = calculateFeedforward(setpointRps * 2.0 * Math.PI, inputs.positionRad);
        double voltage = Math.max(-12.0, Math.min(12.0, simVelocityKp * (setpointRps - velRps) + ff));
        motor.setControl(voltageRequest.withOutput(voltage * powerConstraint.feedforwardCap));

        Logger.recordOutput(logPrefix + "Sim/VelocitySetpointRPS", setpointRps);
        Logger.recordOutput(logPrefix + "Sim/VoltageOutput", voltage);
    }

    private double calculateFeedforward(double velocityRadPerSec, double positionRad) {
        if (!Double.isNaN(feedforwardOverrideVolts)) {
            double ff = feedforwardOverrideVolts;
            feedforwardOverrideVolts = Double.NaN;
            return ff;
        }
        ArmFeedforward armFF = (isSimulation && simArmFeedforward != null)
                ? simArmFeedforward : armFeedforward;
        if (armFF != null) {
            return armFF.calculate(positionRad - horizontalZeroRad, velocityRadPerSec);
        }
        ElevatorFeedforward elevFF = (isSimulation && simElevatorFeedforward != null)
                ? simElevatorFeedforward : elevatorFeedforward;
        if (elevFF != null && mechanismCircumference != null) {
            double linearVelocityMps = velocityRadPerSec / (2.0 * Math.PI)
                    * mechanismCircumference.in(Meters);
            return elevFF.calculate(linearVelocityMps);
        }
        SimpleMotorFeedforward simpleFF = (isSimulation && simSimpleFeedforward != null)
                ? simSimpleFeedforward : simpleFeedforward;
        if (simpleFF != null) {
            return simpleFF.calculate(velocityRadPerSec);
        }
        return 0.0;
    }
}
