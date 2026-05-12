package com.teamscreamrobotics.motorcontrol;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.power.PowerPriority;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;

/** Configuration for a flywheel mechanism. */
public class FlywheelConfig {

    /** The motor this config applies to. */
    public final TalonFXWrapper motor;

    // ── Mechanism / sim ───────────────────────────────────────────────────────

    /** Flywheel wheel diameter — used to compute surface velocity. */
    public Distance diameter = null;
    /** Flywheel mass — required for simulation. */
    public Mass mass = null;
    /** Motor model for simulation. */
    public DCMotor motorModel = null;
    /** Upper velocity soft limit; applied in {@code setVelocity()}. */
    public AngularVelocity upperSoftLimit = null;
    /** Lower velocity soft limit; applied in {@code setVelocity()}. */
    public AngularVelocity lowerSoftLimit = null;
    /** Tolerance for {@code atVelocity()}. Defaults to 50 RPM when null. */
    public AngularVelocity velocityTolerance = null;
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
    private NeutralModeValue neutralMode = NeutralModeValue.Coast;
    private InvertedValue inverted = InvertedValue.CounterClockwise_Positive;
    private double supplyCurrentLimitAmps = 40.0;
    private boolean enableSupplyCurrentLimit = false;
    private double statorCurrentLimitAmps = 80.0;
    private boolean enableStatorCurrentLimit = true;
    private double openLoopRampRate = 0.0;
    private double closedLoopRampRate = 0.0;
    private SimpleMotorFeedforward simpleFeedforward = null;
    private SimpleMotorFeedforward simSimpleFeedforward = null;
    private PowerPriority powerPriority = PowerPriority.MEDIUM;
    private int[] pdhChannels = new int[0];
    private Consumer<TalonFXConfiguration> extraConfigsConsumer = null;
    private Consumer<TalonFXConfiguration> extraSimConfigsConsumer = null;

    public FlywheelConfig(TalonFXWrapper motor) {
        this.motor = motor;
    }

    // ── Mechanism configuration ───────────────────────────────────────────────

    /** Sets the wheel diameter used to compute surface velocity. */
    public FlywheelConfig withDiameter(Distance diameter) {
        this.diameter = diameter;
        return this;
    }

    /** Sets the flywheel mass for simulation. Required for simulation. */
    public FlywheelConfig withMass(Mass mass) {
        this.mass = mass;
        return this;
    }

    /** Sets the motor model used for simulation. */
    public FlywheelConfig withMotorModel(DCMotor model) {
        this.motorModel = model;
        return this;
    }

    /** Sets the maximum velocity the flywheel will accept in {@code setVelocity()}. */
    public FlywheelConfig withUpperSoftLimit(AngularVelocity limit) {
        this.upperSoftLimit = limit;
        return this;
    }

    /** Sets the minimum velocity the flywheel will accept in {@code setVelocity()}. */
    public FlywheelConfig withLowerSoftLimit(AngularVelocity limit) {
        this.lowerSoftLimit = limit;
        return this;
    }

    /** Overrides the default 50 RPM tolerance used by {@code atVelocity()}. */
    public FlywheelConfig withVelocityTolerance(AngularVelocity tolerance) {
        this.velocityTolerance = tolerance;
        return this;
    }

    /** Overrides the default AdvantageKit log prefix for this mechanism. */
    public FlywheelConfig withLogPrefix(String prefix) {
        this.logPrefix = prefix;
        return this;
    }

    // ── Motor configuration ───────────────────────────────────────────────────

    /**
     * Sets the overall gear reduction (sensor-to-mechanism ratio).
     * Use {@link SmartMechanism#gearing(double...)} to compute multi-stage reductions.
     */
    public FlywheelConfig withGearing(double reduction) {
        this.gearing = reduction;
        return this;
    }

    /** Sets PID gains for velocity control (Slot 0). */
    public FlywheelConfig withClosedLoopController(double kP, double kI, double kD) {
        this.slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
        return this;
    }

    /** Sets PID gains and MotionMagic constraints for profiled velocity control. */
    public FlywheelConfig withClosedLoopController(double kP, double kI, double kD,
            AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
        this.slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
        this.motionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(maxVelocity.in(RotationsPerSecond))
                .withMotionMagicAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond));
        return this;
    }

    /**
     * Stores PID gains and MotionMagic constraints intended for simulation. Note: Phoenix 6
     * Slot0 does not run in WPILib simulation — {@link TalonFXWrapper} must manually compute
     * the PID output and send voltage to the motor sim for these gains to have any effect.
     */
    public FlywheelConfig withSimClosedLoopController(double kP, double kI, double kD,
            AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
        this.simSlot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
        this.simMotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(maxVelocity.in(RotationsPerSecond))
                .withMotionMagicAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond));
        return this;
    }

    /** Sets the velocity feedforward (kS, kV) applied during velocity control on real hardware. */
    public FlywheelConfig withFeedforward(SimpleMotorFeedforward ff) {
        this.simpleFeedforward = ff;
        return this;
    }

    /** Sets the velocity feedforward applied in simulation instead of the real feedforward. */
    public FlywheelConfig withSimFeedforward(SimpleMotorFeedforward ff) {
        this.simSimpleFeedforward = ff;
        return this;
    }

    /** Sets the motor neutral mode (brake or coast). Default is coast. */
    public FlywheelConfig withNeutralMode(NeutralModeValue mode) {
        this.neutralMode = mode;
        return this;
    }

    /** Sets the motor output direction. Default is CounterClockwise_Positive. */
    public FlywheelConfig withInverted(InvertedValue invert) {
        this.inverted = invert;
        return this;
    }

    /** Enables and sets the supply (input) current limit. */
    public FlywheelConfig withSupplyCurrentLimit(Current limit) {
        this.supplyCurrentLimitAmps = limit.in(Amps);
        this.enableSupplyCurrentLimit = true;
        return this;
    }

    /** Enables and sets the stator (output) current limit. Enabled by default at 80 A. */
    public FlywheelConfig withStatorCurrentLimit(Current limit) {
        this.statorCurrentLimitAmps = limit.in(Amps);
        this.enableStatorCurrentLimit = true;
        return this;
    }

    /** Sets the power allocation priority used by the power manager. Default is MEDIUM. */
    public FlywheelConfig withPowerPriority(PowerPriority priority) {
        this.powerPriority = priority;
        return this;
    }

    /**
     * Applied as the final step before passing the configuration to the motor.
     * Use for any TalonFX settings not covered by the fluent API.
     */
    public FlywheelConfig withExtraConfigs(Consumer<TalonFXConfiguration> consumer) {
        this.extraConfigsConsumer = consumer;
        return this;
    }

    /** Same as {@link #withExtraConfigs} but applied to the simulation configuration. */
    public FlywheelConfig withExtraSimConfigs(Consumer<TalonFXConfiguration> consumer) {
        this.extraSimConfigsConsumer = consumer;
        return this;
    }

    /** MOI for a solid disc: 0.5 * mass * radius². */
    public double getMOI() {
        if (diameter == null || mass == null)
            return 0.001;
        double r = diameter.in(Meters) / 2.0;
        return 0.5 * mass.in(Kilograms) * r * r;
    }

    // ── Internal ──────────────────────────────────────────────────────────────

    void applyBuilt() {
        TalonFXConfiguration real = buildTalonFXConfig();
        TalonFXConfiguration sim = (simSlot0 != null) ? buildSimTalonFXConfig(real) : null;

        if (extraConfigsConsumer != null)
            extraConfigsConsumer.accept(real);
        if (extraSimConfigsConsumer != null) {
            if (sim == null)
                sim = cloneConfig(real);
            extraSimConfigsConsumer.accept(sim);
        }

        motor.applyMechanismConfig(new MechanismApplyConfig(
                resolveLogPrefix(), real, sim, null, -1, "", false, null,
                null, null, null, null, simpleFeedforward, simSimpleFeedforward,
                0.0, powerPriority, pdhChannels));
    }

    public String resolveLogPrefix() {
        if (logPrefix != null)
            return logPrefix;
        return "Mechanisms/FlyWheel/" + motor.getRuntimeInfo().subsystem().getClass().getSimpleName() + "/";
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
        return cfg;
    }

    private TalonFXConfiguration buildSimTalonFXConfig(TalonFXConfiguration base) {
        TalonFXConfiguration sim = cloneConfig(base);
        sim.Slot0 = simSlot0;
        if (simMotionMagic != null)
            sim.MotionMagic = simMotionMagic;
        return sim;
    }

    private static TalonFXConfiguration cloneConfig(TalonFXConfiguration src) {
        TalonFXConfiguration dst = new TalonFXConfiguration();
        dst.MotorOutput = src.MotorOutput;
        dst.CurrentLimits = src.CurrentLimits;
        dst.OpenLoopRamps = src.OpenLoopRamps;
        dst.ClosedLoopRamps = src.ClosedLoopRamps;
        dst.Feedback = src.Feedback;
        dst.Slot0 = src.Slot0;
        dst.MotionMagic = src.MotionMagic;
        return dst;
    }
}
