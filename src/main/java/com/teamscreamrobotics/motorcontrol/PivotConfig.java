package com.teamscreamrobotics.motorcontrol;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.power.PowerPriority;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;

/** Configuration for a gravity-affected pivot mechanism. */
public class PivotConfig {

    /** The motor this config applies to. */
    public final TalonFXWrapper motor;

    // ── Sim physics ───────────────────────────────────────────────────────────

    /** Motor model for simulation. */
    public DCMotor motorModel = null;
    /** Moment of inertia (kg·m²) — used for DCMotorSim. */
    public double moiKgMetersSquared = 0.001;
    /** Minimum angle hard stop — used only as a sim physics bound. */
    public Rotation2d hardLimitMin = null;
    /** Maximum angle hard stop — used only as a sim physics bound. */
    public Rotation2d hardLimitMax = null;
    /** Starting encoder position seeded on construction. */
    public Rotation2d startingPosition = new Rotation2d();
    /** Wrapping range minimum — non-null only when continuous wrap is enabled. */
    public Rotation2d wrappingMin = null;
    /** Wrapping range maximum — non-null only when continuous wrap is enabled. */
    public Rotation2d wrappingMax = null;
    /** Tolerance for {@code atAngle()}. Defaults to 1 degree when null. */
    public Rotation2d positionTolerance = null;
    public String logPrefix = null;

    // ── Internal motor config state ───────────────────────────────────────────

    private double gearing = 1.0;
    private double rotorToSensorRatio = 1.0;
    private FeedbackSensorSourceValue feedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    private int feedbackRemoteSensorId = 0;
    private Slot0Configs slot0 = new Slot0Configs();
    private MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    private Slot0Configs simSlot0 = null;
    private MotionMagicConfigs simMotionMagic = null;
    private NeutralModeValue neutralMode = NeutralModeValue.Brake;
    private InvertedValue inverted = InvertedValue.CounterClockwise_Positive;
    private double supplyCurrentLimitAmps = 40.0;
    private boolean enableSupplyCurrentLimit = false;
    private double statorCurrentLimitAmps = 80.0;
    private boolean enableStatorCurrentLimit = true;
    private double openLoopRampRate = 0.0;
    private double closedLoopRampRate = 0.0;
    private Rotation2d forwardSoftLimit = null;
    private Rotation2d reverseSoftLimit = null;
    private boolean continuousWrap = false;
    private int cancoderCanId = -1;
    private String cancoderCanbus = "";
    private double cancoderOffsetRotations = 0.0;
    private boolean cancoderSeedPosition = true;
    private ArmFeedforward armFeedforward = null;
    private ArmFeedforward simArmFeedforward = null;
    private PowerPriority powerPriority = PowerPriority.MEDIUM;
    private int[] pdhChannels = new int[0];
    private Consumer<TalonFXConfiguration> extraConfigsConsumer = null;
    private Consumer<TalonFXConfiguration> extraSimConfigsConsumer = null;
    private Consumer<CANcoderConfiguration> extraCANcoderConfigsConsumer = null;

    public PivotConfig(TalonFXWrapper motor) {
        this.motor = motor;
    }

    // ── Mechanism configuration ───────────────────────────────────────────────

    /** Sets the motor model used for simulation. */
    public PivotConfig withMotorModel(DCMotor model) { this.motorModel = model; return this; }

    /** Sets moment of inertia from physical dimensions. */
    public PivotConfig withMOI(double kgMetersSquared) {
        this.moiKgMetersSquared = kgMetersSquared; return this;
    }

    /** Convenience: computes disc MOI from radius and mass. */
    public PivotConfig withMOI(Distance radius, Mass mass) {
        this.moiKgMetersSquared = 0.5 * mass.in(Kilograms) * Math.pow(radius.in(Meters), 2);
        return this;
    }

    /** Sets physical hard-stop angles used as simulation bounds. */
    public PivotConfig withHardLimit(Rotation2d min, Rotation2d max) {
        this.hardLimitMin = min; this.hardLimitMax = max; return this;
    }

    /** Enables TalonFX software limit switches at the given angles. */
    public PivotConfig withSoftLimits(Rotation2d min, Rotation2d max) {
        this.reverseSoftLimit = min; this.forwardSoftLimit = max; return this;
    }

    /** Seeds the encoder to this angle when the mechanism is constructed. */
    public PivotConfig withStartingPosition(Rotation2d position) {
        this.startingPosition = position; return this;
    }

    /**
     * Enables ContinuousWrap in ClosedLoopGeneralConfigs and stores the wrapping range
     * (used for angle normalization in {@link Turret#track}).
     */
    public PivotConfig withWrapping(Rotation2d min, Rotation2d max) {
        this.wrappingMin = min; this.wrappingMax = max;
        this.continuousWrap = true;
        return this;
    }

    /** Overrides the default 1-degree tolerance used by {@code atAngle()}. */
    public PivotConfig withPositionTolerance(Rotation2d tolerance) {
        this.positionTolerance = tolerance; return this;
    }

    /** Overrides the default AdvantageKit log prefix for this mechanism. */
    public PivotConfig withLogPrefix(String prefix) { this.logPrefix = prefix; return this; }

    // ── Motor configuration ───────────────────────────────────────────────────

    /**
     * Sets the overall gear reduction (sensor-to-mechanism ratio).
     * Use {@link SmartMechanism#gearing(double...)} to compute multi-stage reductions.
     */
    public PivotConfig withGearing(double reduction) { this.gearing = reduction; return this; }

    /** Sets the rotor-to-sensor ratio. Only needed when using a remote sensor (CANcoder). */
    public PivotConfig withRotorToSensorRatio(double ratio) {
        this.rotorToSensorRatio = ratio; return this;
    }

    /**
     * Selects the feedback sensor source and remote sensor ID.
     * For FusedCANcoder: {@code withFeedbackType(FeedbackSensorSourceValue.FusedCANcoder, cancoderId)}.
     */
    public PivotConfig withFeedbackType(FeedbackSensorSourceValue source, int remoteSensorId) {
        this.feedbackSensorSource = source; this.feedbackRemoteSensorId = remoteSensorId; return this;
    }

    /** Sets PID gains for direct position control (Slot 0). */
    public PivotConfig withClosedLoopController(double kP, double kI, double kD) {
        this.slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD); return this;
    }

    /** Sets PID gains and MotionMagic constraints for profiled position control. */
    public PivotConfig withClosedLoopController(double kP, double kI, double kD,
            AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
        this.slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
        this.motionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(maxVelocity.in(RotationsPerSecond))
                .withMotionMagicAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond));
        return this;
    }

    /**
     * Stores PID gains intended for simulation. Note: Phoenix 6 Slot0 does not run in WPILib
     * simulation — {@link TalonFXWrapper} must manually compute the PID output and send voltage
     * to the motor sim for these gains to have any effect.
     */
    public PivotConfig withSimClosedLoopController(double kP, double kI, double kD) {
        this.simSlot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD); return this;
    }

    /**
     * Stores PID gains and MotionMagic constraints intended for simulation. Note: Phoenix 6
     * Slot0 does not run in WPILib simulation — {@link TalonFXWrapper} must manually compute
     * the PID output and send voltage to the motor sim for these gains to have any effect.
     */
    public PivotConfig withSimClosedLoopController(double kP, double kI, double kD,
            AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
        this.simSlot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
        this.simMotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(maxVelocity.in(RotationsPerSecond))
                .withMotionMagicAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond));
        return this;
    }

    /** Sets the arm feedforward (kS, kG, kV) applied during position control on real hardware. */
    public PivotConfig withFeedforward(ArmFeedforward ff) { this.armFeedforward = ff; return this; }
    /** Sets the arm feedforward applied in simulation instead of the real feedforward. */
    public PivotConfig withSimFeedforward(ArmFeedforward ff) { this.simArmFeedforward = ff; return this; }

    /** Sets the motor neutral mode (brake or coast). Default is brake. */
    public PivotConfig withNeutralMode(NeutralModeValue mode) { this.neutralMode = mode; return this; }
    /** Sets the motor output direction. Default is CounterClockwise_Positive. */
    public PivotConfig withInverted(InvertedValue invert) { this.inverted = invert; return this; }

    /** Enables and sets the supply (input) current limit. */
    public PivotConfig withSupplyCurrentLimit(Current limit) {
        this.supplyCurrentLimitAmps = limit.in(Amps); this.enableSupplyCurrentLimit = true; return this;
    }

    /** Enables and sets the stator (output) current limit. Enabled by default at 80 A. */
    public PivotConfig withStatorCurrentLimit(Current limit) {
        this.statorCurrentLimitAmps = limit.in(Amps); this.enableStatorCurrentLimit = true; return this;
    }

    /** Sets the open-loop voltage ramp rate in seconds (0 = no ramp). */
    public PivotConfig withOpenLoopRampRate(double seconds) { this.openLoopRampRate = seconds; return this; }
    /** Sets the closed-loop voltage ramp rate in seconds (0 = no ramp). */
    public PivotConfig withClosedLoopRampRate(double seconds) { this.closedLoopRampRate = seconds; return this; }

    /**
     * Configures a FusedCANcoder on the default CAN bus.
     * Sets {@code FeedbackSensorSource = FusedCANcoder} and {@code FeedbackRemoteSensorID = canId}.
     * Call {@link #withCANcoderOffset} to set the magnet offset.
     */
    public PivotConfig withCANcoder(int canId) {
        this.cancoderCanId = canId;
        this.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        this.feedbackRemoteSensorId = canId;
        return this;
    }

    /** {@link #withCANcoder(int)} with an explicit CAN bus. */
    public PivotConfig withCANcoder(int canId, String canbus) {
        this.cancoderCanId = canId; this.cancoderCanbus = canbus;
        this.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        this.feedbackRemoteSensorId = canId;
        return this;
    }

    /** Sets the CANcoder magnet offset (rotations). */
    public PivotConfig withCANcoderOffset(double offsetRotations) {
        this.cancoderOffsetRotations = offsetRotations; return this;
    }

    /** Sets the power allocation priority used by the power manager. Default is MEDIUM. */
    public PivotConfig withPowerPriority(PowerPriority priority) {
        this.powerPriority = priority; return this;
    }

    /**
     * Applied as the final step before passing the configuration to the motor.
     * Use for any TalonFX settings not covered by the fluent API.
     */
    public PivotConfig withExtraConfigs(Consumer<TalonFXConfiguration> consumer) {
        this.extraConfigsConsumer = consumer; return this;
    }

    /** Same as {@link #withExtraConfigs} but applied to the simulation configuration. */
    public PivotConfig withExtraSimConfigs(Consumer<TalonFXConfiguration> consumer) {
        this.extraSimConfigsConsumer = consumer; return this;
    }

    /** Applied to the CANcoder configuration after {@link #withCANcoder(int)} is called. */
    public PivotConfig withExtraCANcoderConfigs(Consumer<CANcoderConfiguration> consumer) {
        this.extraCANcoderConfigsConsumer = consumer; return this;
    }

    // ── Internal ──────────────────────────────────────────────────────────────

    void applyBuilt() {
        TalonFXConfiguration real = buildTalonFXConfig();
        TalonFXConfiguration sim = (simSlot0 != null) ? buildSimTalonFXConfig(real) : null;
        CANcoderConfiguration cancoderCfg = (cancoderCanId >= 0) ? buildCANcoderConfig() : null;

        if (extraConfigsConsumer != null) extraConfigsConsumer.accept(real);
        if (extraSimConfigsConsumer != null) {
            if (sim == null) sim = cloneConfig(real);
            extraSimConfigsConsumer.accept(sim);
        }
        if (extraCANcoderConfigsConsumer != null) {
            if (cancoderCfg == null) cancoderCfg = new CANcoderConfiguration();
            extraCANcoderConfigsConsumer.accept(cancoderCfg);
        }

        motor.applyMechanismConfig(new MechanismApplyConfig(
                resolveLogPrefix(), real, sim, cancoderCfg, cancoderCanId, cancoderCanbus,
                cancoderSeedPosition, null,
                armFeedforward, simArmFeedforward, null, null, null, null,
                0.0, powerPriority, pdhChannels));
    }

    public String resolveLogPrefix() {
        if (logPrefix != null) return logPrefix;
        return "Mechanisms/Pivot/" + motor.getRuntimeInfo().subsystem().getClass().getSimpleName() + "/";
    }

    private TalonFXConfiguration buildTalonFXConfig() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput = new MotorOutputConfigs().withInverted(inverted).withNeutralMode(neutralMode);
        cfg.CurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(supplyCurrentLimitAmps)
                .withSupplyCurrentLimitEnable(enableSupplyCurrentLimit)
                .withStatorCurrentLimit(statorCurrentLimitAmps)
                .withStatorCurrentLimitEnable(enableStatorCurrentLimit);
        cfg.OpenLoopRamps = new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(openLoopRampRate);
        cfg.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(closedLoopRampRate);
        cfg.Feedback = new FeedbackConfigs()
                .withSensorToMechanismRatio(gearing)
                .withRotorToSensorRatio(rotorToSensorRatio)
                .withFeedbackSensorSource(feedbackSensorSource)
                .withFeedbackRemoteSensorID(feedbackRemoteSensorId);
        cfg.Slot0 = slot0;
        cfg.MotionMagic = motionMagic;
        if (continuousWrap) {
            cfg.ClosedLoopGeneral = new ClosedLoopGeneralConfigs().withContinuousWrap(true);
        }
        if (forwardSoftLimit != null || reverseSoftLimit != null) {
            SoftwareLimitSwitchConfigs sl = new SoftwareLimitSwitchConfigs();
            if (forwardSoftLimit != null)
                sl.withForwardSoftLimitThreshold(forwardSoftLimit.getRotations()).withForwardSoftLimitEnable(true);
            if (reverseSoftLimit != null)
                sl.withReverseSoftLimitThreshold(reverseSoftLimit.getRotations()).withReverseSoftLimitEnable(true);
            cfg.SoftwareLimitSwitch = sl;
        }
        return cfg;
    }

    private TalonFXConfiguration buildSimTalonFXConfig(TalonFXConfiguration base) {
        TalonFXConfiguration sim = cloneConfig(base);
        sim.Slot0 = simSlot0;
        if (simMotionMagic != null) sim.MotionMagic = simMotionMagic;
        return sim;
    }

    private CANcoderConfiguration buildCANcoderConfig() {
        CANcoderConfiguration cfg = new CANcoderConfiguration();
        cfg.MagnetSensor = new MagnetSensorConfigs().withMagnetOffset(cancoderOffsetRotations);
        return cfg;
    }

    private static TalonFXConfiguration cloneConfig(TalonFXConfiguration src) {
        TalonFXConfiguration dst = new TalonFXConfiguration();
        dst.MotorOutput = src.MotorOutput; dst.CurrentLimits = src.CurrentLimits;
        dst.OpenLoopRamps = src.OpenLoopRamps; dst.ClosedLoopRamps = src.ClosedLoopRamps;
        dst.Feedback = src.Feedback; dst.Slot0 = src.Slot0; dst.MotionMagic = src.MotionMagic;
        dst.SoftwareLimitSwitch = src.SoftwareLimitSwitch;
        dst.ClosedLoopGeneral = src.ClosedLoopGeneral;
        return dst;
    }
}
