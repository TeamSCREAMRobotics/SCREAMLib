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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;

/** Configuration for a linear elevator mechanism. */
public class ElevatorConfig {

    /** The motor this config applies to. */
    public final TalonFXWrapper motor;

    // ── Sim physics ───────────────────────────────────────────────────────────

    /** Total carriage mass. Required for simulation. */
    public Mass mass = null;
    /** Motor model for simulation. */
    public DCMotor motorModel = null;
    /** Minimum height hard stop — used only as a sim physics bound. */
    public Distance hardLimitMin = null;
    /** Maximum height hard stop — used only as a sim physics bound. */
    public Distance hardLimitMax = null;
    /** Starting encoder height seeded on construction. */
    public Distance startingHeight = Meters.of(0);
    /** Drum or sprocket circumference used to convert rotations to linear distance. */
    public Distance mechanismCircumference = null;
    /** Tolerance for {@code atHeight()}. Defaults to 1 cm when null. */
    public Distance positionTolerance = null;
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
    private Angle forwardSoftLimit = null;
    private Angle reverseSoftLimit = null;
    private Distance softLimitMin = null;
    private Distance softLimitMax = null;
    private int cancoderCanId = -1;
    private String cancoderCanbus = "";
    private double cancoderOffsetRotations = 0.0;
    private boolean cancoderSeedPosition = true;
    private ElevatorFeedforward elevatorFeedforward = null;
    private ElevatorFeedforward simElevatorFeedforward = null;
    private PowerPriority powerPriority = PowerPriority.MEDIUM;
    private int[] pdhChannels = new int[0];
    private Consumer<TalonFXConfiguration> extraConfigsConsumer = null;
    private Consumer<TalonFXConfiguration> extraSimConfigsConsumer = null;
    private Consumer<CANcoderConfiguration> extraCANcoderConfigsConsumer = null;

    public ElevatorConfig(TalonFXWrapper motor) {
        this.motor = motor;
    }

    // ── Mechanism configuration ───────────────────────────────────────────────

    /** Sets the total carriage mass. Required for simulation. */
    public ElevatorConfig withMass(Mass mass) { this.mass = mass; return this; }
    /** Sets the motor model used for simulation. */
    public ElevatorConfig withMotorModel(DCMotor model) { this.motorModel = model; return this; }

    /** Sets physical hard-stop heights used as simulation bounds. */
    public ElevatorConfig withHardLimits(Distance min, Distance max) {
        this.hardLimitMin = min; this.hardLimitMax = max; return this;
    }

    /**
     * Sets software limit switches in meters.
     * {@link #withMechanismCircumference} must be called before this method.
     */
    public ElevatorConfig withSoftLimits(Distance min, Distance max) {
        this.softLimitMin = min; this.softLimitMax = max; return this;
    }

    /** Seeds the encoder to this height when the mechanism is constructed. */
    public ElevatorConfig withStartingHeight(Distance height) {
        this.startingHeight = height; return this;
    }

    /** Overrides the default 1 cm tolerance used by {@code atHeight()}. */
    public ElevatorConfig withPositionTolerance(Distance tolerance) {
        this.positionTolerance = tolerance; return this;
    }

    /** Overrides the default AdvantageKit log prefix for this mechanism. */
    public ElevatorConfig withLogPrefix(String prefix) { this.logPrefix = prefix; return this; }

    // ── Motor configuration ───────────────────────────────────────────────────

    /**
     * Sets the overall gear reduction (sensor-to-mechanism ratio).
     * Use {@link SmartMechanism#gearing(double...)} to compute multi-stage reductions.
     */
    public ElevatorConfig withGearing(double reduction) { this.gearing = reduction; return this; }

    /** Sets the rotor-to-sensor ratio. Only needed when using a remote sensor (CANcoder). */
    public ElevatorConfig withRotorToSensorRatio(double ratio) {
        this.rotorToSensorRatio = ratio; return this;
    }

    /**
     * Selects the feedback sensor source and remote sensor ID.
     * For FusedCANcoder: {@code withFeedbackType(FeedbackSensorSourceValue.FusedCANcoder, cancoderId)}.
     */
    public ElevatorConfig withFeedbackType(FeedbackSensorSourceValue source, int remoteSensorId) {
        this.feedbackSensorSource = source; this.feedbackRemoteSensorId = remoteSensorId; return this;
    }

    /**
     * Sets the drum or sprocket circumference used to convert motor rotations to linear distance.
     * Must be called before {@link #withSoftLimits}.
     */
    public ElevatorConfig withMechanismCircumference(Distance circumference) {
        this.mechanismCircumference = circumference; return this;
    }

    /** Sets PID gains for direct position control (Slot 0). */
    public ElevatorConfig withClosedLoopController(double kP, double kI, double kD) {
        this.slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD); return this;
    }

    /** Sets PID gains and MotionMagic constraints for profiled position control. */
    public ElevatorConfig withClosedLoopController(double kP, double kI, double kD,
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
    public ElevatorConfig withSimClosedLoopController(double kP, double kI, double kD) {
        this.simSlot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD); return this;
    }

    /**
     * Stores PID gains and MotionMagic constraints intended for simulation. Note: Phoenix 6
     * Slot0 does not run in WPILib simulation — {@link TalonFXWrapper} must manually compute
     * the PID output and send voltage to the motor sim for these gains to have any effect.
     */
    public ElevatorConfig withSimClosedLoopController(double kP, double kI, double kD,
            AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
        this.simSlot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
        this.simMotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(maxVelocity.in(RotationsPerSecond))
                .withMotionMagicAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond));
        return this;
    }

    /** Sets the elevator feedforward (kS, kG, kV) applied during position control on real hardware. */
    public ElevatorConfig withFeedforward(ElevatorFeedforward ff) {
        this.elevatorFeedforward = ff; return this;
    }

    /** Sets the elevator feedforward applied in simulation instead of the real feedforward. */
    public ElevatorConfig withSimFeedforward(ElevatorFeedforward ff) {
        this.simElevatorFeedforward = ff; return this;
    }

    /** Sets the motor neutral mode (brake or coast). Default is brake. */
    public ElevatorConfig withNeutralMode(NeutralModeValue mode) { this.neutralMode = mode; return this; }
    /** Sets the motor output direction. Default is CounterClockwise_Positive. */
    public ElevatorConfig withInverted(InvertedValue invert) { this.inverted = invert; return this; }

    /** Enables and sets the supply (input) current limit. */
    public ElevatorConfig withSupplyCurrentLimit(Current limit) {
        this.supplyCurrentLimitAmps = limit.in(Amps); this.enableSupplyCurrentLimit = true; return this;
    }

    /** Enables and sets the stator (output) current limit. Enabled by default at 80 A. */
    public ElevatorConfig withStatorCurrentLimit(Current limit) {
        this.statorCurrentLimitAmps = limit.in(Amps); this.enableStatorCurrentLimit = true; return this;
    }

    /** Sets the open-loop voltage ramp rate in seconds (0 = no ramp). */
    public ElevatorConfig withOpenLoopRampRate(double seconds) {
        this.openLoopRampRate = seconds; return this;
    }

    /** Sets the closed-loop voltage ramp rate in seconds (0 = no ramp). */
    public ElevatorConfig withClosedLoopRampRate(double seconds) {
        this.closedLoopRampRate = seconds; return this;
    }

    /**
     * Configures a FusedCANcoder on the default CAN bus.
     * Sets {@code FeedbackSensorSource = FusedCANcoder} and {@code FeedbackRemoteSensorID = canId}.
     * Call {@link #withCANcoderOffset} to set the magnet offset.
     */
    public ElevatorConfig withCANcoder(int canId) {
        this.cancoderCanId = canId;
        this.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        this.feedbackRemoteSensorId = canId;
        return this;
    }

    /** {@link #withCANcoder(int)} with an explicit CAN bus. */
    public ElevatorConfig withCANcoder(int canId, String canbus) {
        this.cancoderCanId = canId; this.cancoderCanbus = canbus;
        this.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        this.feedbackRemoteSensorId = canId;
        return this;
    }

    /** Sets the CANcoder magnet offset (rotations). */
    public ElevatorConfig withCANcoderOffset(double offsetRotations) {
        this.cancoderOffsetRotations = offsetRotations; return this;
    }

    /** Sets the power allocation priority used by the power manager. Default is MEDIUM. */
    public ElevatorConfig withPowerPriority(PowerPriority priority) {
        this.powerPriority = priority; return this;
    }

    /**
     * Applied as the final step before passing the configuration to the motor.
     * Use for any TalonFX settings not covered by the fluent API.
     */
    public ElevatorConfig withExtraConfigs(Consumer<TalonFXConfiguration> consumer) {
        this.extraConfigsConsumer = consumer; return this;
    }

    /** Same as {@link #withExtraConfigs} but applied to the simulation configuration. */
    public ElevatorConfig withExtraSimConfigs(Consumer<TalonFXConfiguration> consumer) {
        this.extraSimConfigsConsumer = consumer; return this;
    }

    /** Applied to the CANcoder configuration after {@link #withCANcoder(int)} is called. */
    public ElevatorConfig withExtraCANcoderConfigs(Consumer<CANcoderConfiguration> consumer) {
        this.extraCANcoderConfigsConsumer = consumer; return this;
    }

    // ── Internal ──────────────────────────────────────────────────────────────

    void applyBuilt() {
        // Convert distance soft limits to rotations if circumference is set
        if (softLimitMin != null && mechanismCircumference != null) {
            double circ = mechanismCircumference.in(Meters);
            reverseSoftLimit = Rotations.of(softLimitMin.in(Meters) / circ);
            forwardSoftLimit = Rotations.of(softLimitMax.in(Meters) / circ);
        }

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
                cancoderSeedPosition, mechanismCircumference,
                null, null, elevatorFeedforward, simElevatorFeedforward, null, null,
                0.0, powerPriority, pdhChannels));
    }

    public String resolveLogPrefix() {
        if (logPrefix != null) return logPrefix;
        return "Mechanisms/Elevator/" + motor.getRuntimeInfo().subsystem().getClass().getSimpleName() + "/";
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
                sl.withForwardSoftLimitThreshold(forwardSoftLimit.in(Rotations)).withForwardSoftLimitEnable(true);
            if (reverseSoftLimit != null)
                sl.withReverseSoftLimitThreshold(reverseSoftLimit.in(Rotations)).withReverseSoftLimitEnable(true);
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
        return dst;
    }
}
