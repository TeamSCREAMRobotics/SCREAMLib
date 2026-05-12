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
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

/** Configuration for a turret mechanism with optional CRT absolute positioning. */
public class TurretConfig {

    /** The motor this config applies to. */
    public final TalonFXWrapper motor;

    // ── Sim physics ───────────────────────────────────────────────────────────

    /** Motor model for simulation. */
    public DCMotor motorModel = null;
    /** Moment of inertia (kg·m²) — used for DCMotorSim. */
    public double moiKgMetersSquared = 0.001;
    /** Physical hard-stop angles. Also used by {@link Turret#track} to normalize into range. */
    public Rotation2d hardLimitMin = null;
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

    /**
     * Dual-encoder absolute positioning via the Chinese Remainder Theorem.
     * If null, the turret uses its starting position as the initial encoder value.
     */
    public CRTConfig crtConfig = null;

    /** Optional lag compensation. See {@link #withLagCompensation}. */
    public Supplier<AngularVelocity> drivetrainAngularVelocitySupplier = null;
    public double lagCompensationSeconds = 0.0;

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

    // ── CRT config ────────────────────────────────────────────────────────────

    /**
     * Parameters for two-CANcoder Chinese Remainder Theorem absolute positioning.
     *
     * <p>The primary encoder should be the <b>fine</b> (faster-turning) one. The secondary
     * should be the <b>coarse</b> (slower-turning) one.
     */
    public record CRTConfig(
            int primaryCanId,
            String primaryCanbus,
            double primaryMagnetOffsetRotations,
            double primaryTurnsPerMechanismTurn,
            int secondaryCanId,
            String secondaryCanbus,
            double secondaryMagnetOffsetRotations,
            double secondaryTurnsPerMechanismTurn,
            Rotation2d driftWarningThreshold
    ) {
        /** Convenience overload: default CAN bus, 5-degree drift warning threshold. */
        public CRTConfig(
                int primaryCanId, double primaryMagnetOffsetRotations, double primaryTurnsPerMechanismTurn,
                int secondaryCanId, double secondaryMagnetOffsetRotations, double secondaryTurnsPerMechanismTurn) {
            this(primaryCanId, "", primaryMagnetOffsetRotations, primaryTurnsPerMechanismTurn,
                 secondaryCanId, "", secondaryMagnetOffsetRotations, secondaryTurnsPerMechanismTurn,
                 Rotation2d.fromDegrees(5.0));
        }
    }

    // ── Constructor ───────────────────────────────────────────────────────────

    public TurretConfig(TalonFXWrapper motor) {
        this.motor = motor;
    }

    // ── Mechanism configuration ───────────────────────────────────────────────

    /** Sets the motor model used for simulation. */
    public TurretConfig withMotorModel(DCMotor model) { this.motorModel = model; return this; }

    /** Sets moment of inertia from physical dimensions. */
    public TurretConfig withMOI(double kgMetersSquared) {
        this.moiKgMetersSquared = kgMetersSquared; return this;
    }

    /** Convenience: computes disc MOI from radius and mass. */
    public TurretConfig withMOI(Distance radius, Mass mass) {
        this.moiKgMetersSquared = 0.5 * mass.in(Kilograms) * Math.pow(radius.in(Meters), 2);
        return this;
    }

    /** Seeds the encoder to this angle when the mechanism is constructed (non-CRT mode only). */
    public TurretConfig withStartingPosition(Rotation2d position) {
        this.startingPosition = position; return this;
    }

    /** Sets physical hard-stop angles. Also used by {@link Turret#track} to normalize into range. */
    public TurretConfig withHardLimit(Rotation2d min, Rotation2d max) {
        this.hardLimitMin = min; this.hardLimitMax = max; return this;
    }

    /** Enables TalonFX software limit switches at the given angles. */
    public TurretConfig withSoftLimits(Rotation2d min, Rotation2d max) {
        this.reverseSoftLimit = min; this.forwardSoftLimit = max; return this;
    }

    /**
     * Enables ContinuousWrap and stores the wrap range for use by {@link Turret#track}.
     */
    public TurretConfig withWrapping(Rotation2d min, Rotation2d max) {
        this.wrappingMin = min; this.wrappingMax = max;
        this.hardLimitMin = min; this.hardLimitMax = max;
        this.continuousWrap = true;
        return this;
    }

    /** Overrides the default 1-degree tolerance used by {@code atAngle()}. */
    public TurretConfig withPositionTolerance(Rotation2d tolerance) {
        this.positionTolerance = tolerance; return this;
    }

    /** Enables CRT absolute positioning using two CANcoders. See {@link CRTConfig}. */
    public TurretConfig withCRTPositioning(CRTConfig crtConfig) {
        this.crtConfig = crtConfig; return this;
    }

    /**
     * Enables heading-lag feedforward compensation. The applied correction is:
     * <pre>correctedAngle = desiredAngle - drivetrainOmega * lagSeconds</pre>
     */
    public TurretConfig withLagCompensation(
            Supplier<AngularVelocity> drivetrainAngularVelocitySupplier,
            double lagSeconds) {
        this.drivetrainAngularVelocitySupplier = drivetrainAngularVelocitySupplier;
        this.lagCompensationSeconds = lagSeconds;
        return this;
    }

    /** Overrides the default AdvantageKit log prefix for this mechanism. */
    public TurretConfig withLogPrefix(String prefix) { this.logPrefix = prefix; return this; }

    // ── Motor configuration ───────────────────────────────────────────────────

    /**
     * Sets the overall gear reduction (sensor-to-mechanism ratio).
     * Use {@link SmartMechanism#gearing(double...)} to compute multi-stage reductions.
     */
    public TurretConfig withGearing(double reduction) { this.gearing = reduction; return this; }

    /** Sets the rotor-to-sensor ratio. Only needed when using a remote sensor (CANcoder). */
    public TurretConfig withRotorToSensorRatio(double ratio) {
        this.rotorToSensorRatio = ratio; return this;
    }

    /**
     * Selects the feedback sensor source and remote sensor ID.
     * For FusedCANcoder: {@code withFeedbackType(FeedbackSensorSourceValue.FusedCANcoder, cancoderId)}.
     */
    public TurretConfig withFeedbackType(FeedbackSensorSourceValue source, int remoteSensorId) {
        this.feedbackSensorSource = source; this.feedbackRemoteSensorId = remoteSensorId; return this;
    }

    /** Sets PID gains for direct position control (Slot 0). */
    public TurretConfig withClosedLoopController(double kP, double kI, double kD) {
        this.slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD); return this;
    }

    /** Sets PID gains and MotionMagic constraints for profiled position control. */
    public TurretConfig withClosedLoopController(double kP, double kI, double kD,
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
    public TurretConfig withSimClosedLoopController(double kP, double kI, double kD) {
        this.simSlot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD); return this;
    }

    /**
     * Stores PID gains and MotionMagic constraints intended for simulation. Note: Phoenix 6
     * Slot0 does not run in WPILib simulation — {@link TalonFXWrapper} must manually compute
     * the PID output and send voltage to the motor sim for these gains to have any effect.
     */
    public TurretConfig withSimClosedLoopController(double kP, double kI, double kD,
            AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
        this.simSlot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
        this.simMotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(maxVelocity.in(RotationsPerSecond))
                .withMotionMagicAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond));
        return this;
    }

    /** Sets the arm feedforward (kS, kG, kV) applied during position control on real hardware. */
    public TurretConfig withFeedforward(ArmFeedforward ff) { this.armFeedforward = ff; return this; }
    /** Sets the arm feedforward applied in simulation instead of the real feedforward. */
    public TurretConfig withSimFeedforward(ArmFeedforward ff) { this.simArmFeedforward = ff; return this; }

    /** Sets the motor neutral mode (brake or coast). Default is brake. */
    public TurretConfig withNeutralMode(NeutralModeValue mode) { this.neutralMode = mode; return this; }
    /** Sets the motor output direction. Default is CounterClockwise_Positive. */
    public TurretConfig withInverted(InvertedValue invert) { this.inverted = invert; return this; }

    /** Enables and sets the supply (input) current limit. */
    public TurretConfig withSupplyCurrentLimit(Current limit) {
        this.supplyCurrentLimitAmps = limit.in(Amps); this.enableSupplyCurrentLimit = true; return this;
    }

    /** Enables and sets the stator (output) current limit. Enabled by default at 80 A. */
    public TurretConfig withStatorCurrentLimit(Current limit) {
        this.statorCurrentLimitAmps = limit.in(Amps); this.enableStatorCurrentLimit = true; return this;
    }

    /** Sets the open-loop voltage ramp rate in seconds (0 = no ramp). */
    public TurretConfig withOpenLoopRampRate(double seconds) { this.openLoopRampRate = seconds; return this; }
    /** Sets the closed-loop voltage ramp rate in seconds (0 = no ramp). */
    public TurretConfig withClosedLoopRampRate(double seconds) { this.closedLoopRampRate = seconds; return this; }

    /**
     * Configures a FusedCANcoder on the default CAN bus.
     * Sets {@code FeedbackSensorSource = FusedCANcoder} and {@code FeedbackRemoteSensorID = canId}.
     * Call {@link #withCANcoderOffset} to set the magnet offset.
     */
    public TurretConfig withCANcoder(int canId) {
        this.cancoderCanId = canId;
        this.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        this.feedbackRemoteSensorId = canId;
        return this;
    }

    /** {@link #withCANcoder(int)} with an explicit CAN bus. */
    public TurretConfig withCANcoder(int canId, String canbus) {
        this.cancoderCanId = canId; this.cancoderCanbus = canbus;
        this.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        this.feedbackRemoteSensorId = canId;
        return this;
    }

    /** Sets the CANcoder magnet offset (rotations). */
    public TurretConfig withCANcoderOffset(double offsetRotations) {
        this.cancoderOffsetRotations = offsetRotations; return this;
    }

    /** Sets the power allocation priority used by the power manager. Default is MEDIUM. */
    public TurretConfig withPowerPriority(PowerPriority priority) {
        this.powerPriority = priority; return this;
    }

    /**
     * Applied as the final step before passing the configuration to the motor.
     * Use for any TalonFX settings not covered by the fluent API.
     */
    public TurretConfig withExtraConfigs(Consumer<TalonFXConfiguration> consumer) {
        this.extraConfigsConsumer = consumer; return this;
    }

    /** Same as {@link #withExtraConfigs} but applied to the simulation configuration. */
    public TurretConfig withExtraSimConfigs(Consumer<TalonFXConfiguration> consumer) {
        this.extraSimConfigsConsumer = consumer; return this;
    }

    /** Applied to the CANcoder configuration after {@link #withCANcoder(int)} is called. */
    public TurretConfig withExtraCANcoderConfigs(Consumer<CANcoderConfiguration> consumer) {
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
        return "Mechanisms/Turret/" + motor.getRuntimeInfo().subsystem().getClass().getSimpleName() + "/";
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
