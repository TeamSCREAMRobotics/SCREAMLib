package com.teamscreamrobotics.motorcontrol;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.power.PowerPriority;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class SmartMotorControllerConfig {

    public enum ControlMode {
        OPEN_LOOP,
        CLOSED_LOOP
    }

    public static class MechanismGearing {
        private final double gearing;

        private MechanismGearing(double gearing) {
            this.gearing = gearing;
        }

        public static MechanismGearing fromReductionStages(double... stages) {
            double product = 1.0;
            for (double stage : stages) {
                product *= stage;
            }
            return new MechanismGearing(product);
        }

        public double getGearing() {
            return gearing;
        }
    }

    /**
     * Describes a single follower motor.
     *
     * @param canId   CAN device ID of the follower TalonFX.
     * @param canbus  CAN bus name (empty string = default bus).
     * @param oppose  true to run opposite the master direction (e.g. motors
     *                mounted facing each other on the same mechanism).
     */
    public record FollowerConfig(int canId, String canbus, boolean oppose) {
        public FollowerConfig(int canId) {
            this(canId, "", false);
        }
        public FollowerConfig(int canId, boolean oppose) {
            this(canId, "", oppose);
        }
    }

    /**
     * Describes an attached CANcoder used for absolute position feedback.
     *
     * @param canId                  CAN device ID of the CANcoder.
     * @param canbus                 CAN bus name (empty string = default bus).
     * @param magnetOffsetRotations  Offset applied to the raw magnet reading (rotations).
     * @param useAsAbsolutePosition  If true, seeds the TalonFX position from the CANcoder
     *                               absolute position on boot.
     */
    public record CANcoderConfig(int canId, String canbus, double magnetOffsetRotations,
                                 boolean useAsAbsolutePosition) {
        public CANcoderConfig(int canId, double magnetOffsetRotations, boolean useAsAbsolutePosition) {
            this(canId, "", magnetOffsetRotations, useAsAbsolutePosition);
        }
    }

    // Required
    public final SubsystemBase subsystem;

    // Motor behavior
    public boolean inverted = false;
    public NeutralModeValue idleMode = NeutralModeValue.Brake;
    public double supplyCurrentLimitAmps = 40.0;
    public double statorCurrentLimitAmps = 80.0;
    public boolean enableSupplyCurrentLimit = false;
    public boolean enableStatorCurrentLimit = true;
    public double openLoopRampRate = 0.0;
    public double closedLoopRampRate = 0.0;

    // Gearing and measurement
    public double gearing = 1.0;
    public Distance mechanismCircumference = null;

    // Closed loop control
    public Slot0Configs slot0 = new Slot0Configs();
    public MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    public Slot0Configs simSlot0 = null;
    public MotionMagicConfigs simMotionMagic = null;
    public ArmFeedforward armFeedforward = null;
    public ElevatorFeedforward elevatorFeedforward = null;
    public SimpleMotorFeedforward simpleFeedforward = null;
    public ArmFeedforward simArmFeedforward = null;
    public ElevatorFeedforward simElevatorFeedforward = null;
    public SimpleMotorFeedforward simSimpleFeedforward = null;
    public ControlMode controlMode = ControlMode.CLOSED_LOOP;

    // Soft limits
    public Angle forwardSoftLimit = null;
    public Angle reverseSoftLimit = null;

    // Goal tolerances
    public Angle positionTolerance = null;
    public AngularVelocity velocityTolerance = null;

    // Power management
    public PowerPriority powerPriority = PowerPriority.MEDIUM;
    public int[] pdhChannels = new int[0];

    // Motor model (for simulation)
    public DCMotor motorModel = null;

    // Continuous wrap (for pivots)
    public boolean enableContinuousWrap = false;

    // CANcoder
    public CANcoderConfig cancoder = null;
    public boolean usePhoenixPro = false;

    // Followers
    private FollowerConfig[] followers = new FollowerConfig[0];

    // Logging
    public String logPrefix = null;

    public SmartMotorControllerConfig(SubsystemBase subsystem) {
        this.subsystem = subsystem;
    }

    public SmartMotorControllerConfig withMotorInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public SmartMotorControllerConfig withIdleMode(NeutralModeValue idleMode) {
        this.idleMode = idleMode;
        return this;
    }

    public SmartMotorControllerConfig withSupplyCurrentLimit(Current limit) {
        this.supplyCurrentLimitAmps = limit.in(Amps);
        this.enableSupplyCurrentLimit = true;
        return this;
    }

    public SmartMotorControllerConfig withStatorCurrentLimit(Current limit) {
        this.statorCurrentLimitAmps = limit.in(Amps);
        this.enableStatorCurrentLimit = true;
        return this;
    }

    public SmartMotorControllerConfig withOpenLoopRampRate(Time rampRate) {
        this.openLoopRampRate = rampRate.in(Seconds);
        return this;
    }

    public SmartMotorControllerConfig withClosedLoopRampRate(Time rampRate) {
        this.closedLoopRampRate = rampRate.in(Seconds);
        return this;
    }

    public SmartMotorControllerConfig withGearing(MechanismGearing gearing) {
        this.gearing = gearing.getGearing();
        return this;
    }

    public SmartMotorControllerConfig withMechanismCircumference(Distance circumference) {
        this.mechanismCircumference = circumference;
        return this;
    }

    public SmartMotorControllerConfig withClosedLoopController(double kP, double kI, double kD) {
        this.slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
        return this;
    }

    public SmartMotorControllerConfig withClosedLoopController(
            double kP, double kI, double kD,
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration) {
        this.slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
        this.motionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(maxVelocity.in(RotationsPerSecond))
                .withMotionMagicAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond));
        return this;
    }

    public SmartMotorControllerConfig withClosedLoopController(
            double kP, double kI, double kD,
            double kS, double kV, double kA, double kG,
            GravityTypeValue gravityType,
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration) {
        this.slot0 = new Slot0Configs()
                .withKP(kP).withKI(kI).withKD(kD)
                .withKS(kS).withKV(kV).withKA(kA).withKG(kG)
                .withGravityType(gravityType);
        this.motionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(maxVelocity.in(RotationsPerSecond))
                .withMotionMagicAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond));
        return this;
    }

    public SmartMotorControllerConfig withSimClosedLoopController(
            double kP, double kI, double kD,
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration) {
        this.simSlot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
        this.simMotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(maxVelocity.in(RotationsPerSecond))
                .withMotionMagicAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond));
        return this;
    }

    public SmartMotorControllerConfig withFeedforward(ArmFeedforward feedforward) {
        this.armFeedforward = feedforward;
        return this;
    }

    public SmartMotorControllerConfig withFeedforward(ElevatorFeedforward feedforward) {
        this.elevatorFeedforward = feedforward;
        return this;
    }

    public SmartMotorControllerConfig withFeedforward(SimpleMotorFeedforward feedforward) {
        this.simpleFeedforward = feedforward;
        return this;
    }

    public SmartMotorControllerConfig withSimFeedforward(ArmFeedforward feedforward) {
        this.simArmFeedforward = feedforward;
        return this;
    }

    public SmartMotorControllerConfig withSimFeedforward(ElevatorFeedforward feedforward) {
        this.simElevatorFeedforward = feedforward;
        return this;
    }

    public SmartMotorControllerConfig withSimFeedforward(SimpleMotorFeedforward feedforward) {
        this.simSimpleFeedforward = feedforward;
        return this;
    }

    public SmartMotorControllerConfig withControlMode(ControlMode controlMode) {
        this.controlMode = controlMode;
        return this;
    }

    public SmartMotorControllerConfig withSoftLimit(Angle reverse, Angle forward) {
        this.reverseSoftLimit = reverse;
        this.forwardSoftLimit = forward;
        return this;
    }

    public SmartMotorControllerConfig withPositionTolerance(Angle tolerance) {
        this.positionTolerance = tolerance;
        return this;
    }

    public SmartMotorControllerConfig withVelocityTolerance(AngularVelocity tolerance) {
        this.velocityTolerance = tolerance;
        return this;
    }

    public SmartMotorControllerConfig withPowerPriority(PowerPriority priority) {
        this.powerPriority = priority;
        return this;
    }

    public SmartMotorControllerConfig withPDHChannels(int... channels) {
        this.pdhChannels = channels;
        return this;
    }

    public SmartMotorControllerConfig withContinuousWrap(boolean enable) {
        this.enableContinuousWrap = enable;
        return this;
    }

    public SmartMotorControllerConfig withCANcoder(CANcoderConfig cancoder) {
        this.cancoder = cancoder;
        return this;
    }

    public SmartMotorControllerConfig withPhoenixPro(boolean usePhoenixPro) {
        this.usePhoenixPro = usePhoenixPro;
        return this;
    }

    public SmartMotorControllerConfig withMotorModel(DCMotor motorModel) {
        this.motorModel = motorModel;
        return this;
    }

    public SmartMotorControllerConfig withFollowers(FollowerConfig... followers) {
        this.followers = followers;
        return this;
    }

    public FollowerConfig[] getFollowers() {
        return followers;
    }

    public SmartMotorControllerConfig withLogPrefix(String logPrefix) {
        this.logPrefix = logPrefix;
        return this;
    }

    public String resolveLogPrefix(String motorName) {
        String base = (logPrefix != null) ? logPrefix : motorName;
        return "Motors/" + subsystem.getClass().getSimpleName() + "/" + base + "/";
    }

    public Slot0Configs resolveSimSlot0() {
        return simSlot0 != null ? simSlot0 : slot0;
    }

    public MotionMagicConfigs resolveSimMotionMagic() {
        return simMotionMagic != null ? simMotionMagic : motionMagic;
    }
}
