package com.teamscreamrobotics.motorcontrol;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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

/** Configuration for a single-joint arm mechanism. */
public class ArmConfig {

    /** The motor this config applies to. */
    public final TalonFXWrapper motor;

    // ── Sim physics ───────────────────────────────────────────────────────────

    /** Arm link length from pivot to end effector. Required for simulation. */
    public Distance length = null;
    /** Total arm mass. Required for simulation. */
    public Mass mass = null;
    /** Motor model for simulation. */
    public DCMotor motorModel = null;
    /** Minimum angle hard stop — used only as a sim physics bound. */
    public Rotation2d hardLimitMin = null;
    /** Maximum angle hard stop — used only as a sim physics bound. */
    public Rotation2d hardLimitMax = null;
    /** Angle at which the mechanism is considered "horizontal" for feedforward. */
    public Rotation2d horizontalZero = new Rotation2d();
    /** Starting encoder position seeded on construction. */
    public Rotation2d startingPosition = new Rotation2d();
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

    public ArmConfig(TalonFXWrapper motor) {
        this.motor = motor;
    }

    // ── Mechanism configuration ───────────────────────────────────────────────

    /** Sets the link length from pivot to end effector. Required for simulation. */
    public ArmConfig withLength(Distance length) { this.length = length; return this; }
    /** Sets the total arm mass. Required for simulation. */
    public ArmConfig withMass(Mass mass) { this.mass = mass; return this; }
    /** Sets the motor model used for simulation. */
    public ArmConfig withMotorModel(DCMotor model) { this.motorModel = model; return this; }

    /** Sets physical hard-stop angles used as simulation bounds. */
    public ArmConfig withHardLimit(Rotation2d min, Rotation2d max) {
        this.hardLimitMin = min; this.hardLimitMax = max; return this;
    }

    /** Enables TalonFX software limit switches at the given angles. */
    public ArmConfig withSoftLimits(Rotation2d min, Rotation2d max) {
        this.reverseSoftLimit = min; this.forwardSoftLimit = max; return this;
    }

    /** Seeds the encoder to this angle when the mechanism is constructed. */
    public ArmConfig withStartingPosition(Rotation2d position) {
        this.startingPosition = position; return this;
    }

    /** Sets the angle the arm must reach before gravity characterization measures {@code kG}. */
    public ArmConfig withHorizontalZero(Rotation2d angle) {
        this.horizontalZero = angle; return this;
    }

    /** Overrides the default 1-degree tolerance used by {@code atAngle()}. */
    public ArmConfig withPositionTolerance(Rotation2d tolerance) {
        this.positionTolerance = tolerance; return this;
    }

    /** Overrides the default AdvantageKit log prefix for this mechanism. */
    public ArmConfig withLogPrefix(String prefix) { this.logPrefix = prefix; return this; }

    // ── Motor configuration ───────────────────────────────────────────────────

    /**
     * Sets the overall gear reduction (sensor-to-mechanism ratio).
     * Use {@link SmartMechanism#gearing(double...)} to compute multi-stage reductions.
     */
    public ArmConfig withGearing(double reduction) { this.gearing = reduction; return this; }

    /** Sets the rotor-to-sensor ratio. Only needed when using a remote sensor (CANcoder). */
    public ArmConfig withRotorToSensorRatio(double ratio) {
        this.rotorToSensorRatio = ratio; return this;
    }

    /**
     * Selects the feedback sensor source and remote sensor ID.
     * For FusedCANcoder: {@code withFeedbackType(FeedbackSensorSourceValue.FusedCANcoder, cancoderId)}.
     */
    public ArmConfig withFeedbackType(FeedbackSensorSourceValue source, int remoteSensorId) {
        this.feedbackSensorSource = source;
        this.feedbackRemoteSensorId = remoteSensorId;
        return this;
    }

    /** Sets PID gains for direct position control (Slot 0). */
    public ArmConfig withClosedLoopController(double kP, double kI, double kD) {
        this.slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
        return this;
    }

    /** Sets PID gains and MotionMagic constraints for profiled position control. */
    public ArmConfig withClosedLoopController(double kP, double kI, double kD,
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
    public ArmConfig withSimClosedLoopController(double kP, double kI, double kD) {
        this.simSlot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
        return this;
    }

    /**
     * Stores PID gains and MotionMagic constraints intended for simulation. Note: Phoenix 6
     * Slot0 does not run in WPILib simulation — {@link TalonFXWrapper} must manually compute
     * the PID output and send voltage to the motor sim for these gains to have any effect.
     */
    public ArmConfig withSimClosedLoopController(double kP, double kI, double kD,
            AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
        this.simSlot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
        this.simMotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(maxVelocity.in(RotationsPerSecond))
                .withMotionMagicAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond));
        return this;
    }

    /** Sets the arm feedforward (kS, kG, kV) applied during position control on real hardware. */
    public ArmConfig withFeedforward(ArmFeedforward ff) { this.armFeedforward = ff; return this; }
    /** Sets the arm feedforward applied in simulation instead of the real feedforward. */
    public ArmConfig withSimFeedforward(ArmFeedforward ff) { this.simArmFeedforward = ff; return this; }

    /** Sets the motor neutral mode (brake or coast). Default is brake. */
    public ArmConfig withNeutralMode(NeutralModeValue mode) { this.neutralMode = mode; return this; }
    /** Sets the motor output direction. Default is CounterClockwise_Positive. */
    public ArmConfig withInverted(InvertedValue invert) { this.inverted = invert; return this; }

    /** Enables and sets the supply (input) current limit. */
    public ArmConfig withSupplyCurrentLimit(Current limit) {
        this.supplyCurrentLimitAmps = limit.in(Amps);
        this.enableSupplyCurrentLimit = true;
        return this;
    }

    /** Enables and sets the stator (output) current limit. Enabled by default at 80 A. */
    public ArmConfig withStatorCurrentLimit(Current limit) {
        this.statorCurrentLimitAmps = limit.in(Amps);
        this.enableStatorCurrentLimit = true;
        return this;
    }

    /** Sets the open-loop voltage ramp rate in seconds (0 = no ramp). */
    public ArmConfig withOpenLoopRampRate(double seconds) {
        this.openLoopRampRate = seconds; return this;
    }

    /** Sets the closed-loop voltage ramp rate in seconds (0 = no ramp). */
    public ArmConfig withClosedLoopRampRate(double seconds) {
        this.closedLoopRampRate = seconds; return this;
    }

    /**
     * Configures a FusedCANcoder on the default CAN bus.
     * Sets {@code FeedbackSensorSource = FusedCANcoder} and {@code FeedbackRemoteSensorID = canId}.
     * Call {@link #withCANcoderOffset} to set the magnet offset.
     */
    public ArmConfig withCANcoder(int canId) {
        this.cancoderCanId = canId;
        this.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        this.feedbackRemoteSensorId = canId;
        return this;
    }

    /** {@link #withCANcoder(int)} with an explicit CAN bus. */
    public ArmConfig withCANcoder(int canId, String canbus) {
        this.cancoderCanId = canId;
        this.cancoderCanbus = canbus;
        this.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        this.feedbackRemoteSensorId = canId;
        return this;
    }

    /** Sets the CANcoder magnet offset (rotations). */
    public ArmConfig withCANcoderOffset(double offsetRotations) {
        this.cancoderOffsetRotations = offsetRotations; return this;
    }

    /**
     * No-op. Followers are configured at {@link TalonFXWrapper} construction time.
     * Pass followers via the {@code TalonFXWrapper(TalonFX, DCMotor, SubsystemBase, FollowerConfig...)} constructor.
     */
    public ArmConfig withFollowers(FollowerConfig... followers) {
        // Followers are set at TalonFXWrapper construction time; this call is a no-op here.
        // Use the TalonFXWrapper(TalonFX, DCMotor, SubsystemBase, FollowerConfig...) constructor.
        return this;
    }

    /** Sets the power allocation priority used by the power manager. Default is MEDIUM. */
    public ArmConfig withPowerPriority(PowerPriority priority) {
        this.powerPriority = priority; return this;
    }

    /**
     * Applied as the final step before passing the configuration to the motor.
     * Use for any TalonFX settings not covered by the fluent API.
     */
    public ArmConfig withExtraConfigs(Consumer<TalonFXConfiguration> consumer) {
        this.extraConfigsConsumer = consumer; return this;
    }

    /** Same as {@link #withExtraConfigs} but applied to the simulation configuration. */
    public ArmConfig withExtraSimConfigs(Consumer<TalonFXConfiguration> consumer) {
        this.extraSimConfigsConsumer = consumer; return this;
    }

    /** Applied to the CANcoder configuration after {@link #withCANcoder(int)} is called. */
    public ArmConfig withExtraCANcoderConfigs(Consumer<CANcoderConfiguration> consumer) {
        this.extraCANcoderConfigsConsumer = consumer; return this;
    }

    // ── Internal ──────────────────────────────────────────────────────────────

    /** Called by {@link Arm} constructor to apply this config to the motor hardware. */
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
                horizontalZero.getRadians(), powerPriority, pdhChannels));
    }

    public String resolveLogPrefix() {
        if (logPrefix != null) return logPrefix;
        return "Mechanisms/Arm/" + motor.getRuntimeInfo().subsystem().getClass().getSimpleName() + "/";
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
        dst.MotorOutput       = src.MotorOutput;
        dst.CurrentLimits     = src.CurrentLimits;
        dst.OpenLoopRamps     = src.OpenLoopRamps;
        dst.ClosedLoopRamps   = src.ClosedLoopRamps;
        dst.Feedback          = src.Feedback;
        dst.Slot0             = src.Slot0;
        dst.MotionMagic       = src.MotionMagic;
        dst.SoftwareLimitSwitch = src.SoftwareLimitSwitch;
        dst.ClosedLoopGeneral = src.ClosedLoopGeneral;
        return dst;
    }
}
