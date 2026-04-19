package com.teamscreamrobotics.pid;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/** A container class for PID constants, along with additional methods. */
public class ScreamPIDConstants implements Cloneable {

  /** Motion Magic cruise velocity (rot/s), acceleration (rot/s²), and jerk (rot/s³) limits. */
  public record MotionMagicConstants(double cruiseVelocity, double acceleration, int jerk) {}

  /** Feedforward gains (kV, kS, kG, kA) plus the gravity model type for a TalonFX slot. */
  public record FeedforwardConstants(
      double kV, double kS, double kG, double kA, GravityTypeValue gravityType) {
    public FeedforwardConstants(double kV, double kS, double kG, double kA) {
      this(kV, kS, kG, kA, GravityTypeValue.Elevator_Static);
    }

    public FeedforwardConstants() {
      this(0, 0, 0, 0, GravityTypeValue.Elevator_Static);
    }
  }

  private double kP, kI, kD, kF = 0;
  private double period = 0.02;
  private double minOutput = -1;
  private double maxOutput = 1;
  private double integralZone = Double.MAX_VALUE;
  private double maxIntegralAccumulator = Double.POSITIVE_INFINITY;
  private double minIntegralAccumulator = Double.NEGATIVE_INFINITY;

  /** Creates a zeroed {@link ScreamPIDConstants} instance. */
  public ScreamPIDConstants() {}

  /**
   * Creates a {@link ScreamPIDConstants} with PID gains (F defaults to 0).
   *
   * @param p proportional gain
   * @param i integral gain
   * @param d derivative gain
   */
  public ScreamPIDConstants(double p, double i, double d) {
    this.kP = p;
    this.kI = i;
    this.kD = d;
  }

  /**
   * Creates a {@link ScreamPIDConstants} with PIDF gains.
   *
   * @param p proportional gain
   * @param i integral gain
   * @param d derivative gain
   * @param f feedforward gain
   */
  public ScreamPIDConstants(double p, double i, double d, double f) {
    this.kP = p;
    this.kI = i;
    this.kD = d;
    this.kF = f;
  }

  /** Sets P, I, and D gains in place. */
  public void setPID(double p, double i, double d) {
    this.kP = p;
    this.kI = i;
    this.kD = d;
  }

  /** Sets P, I, D, and F gains in place. */
  public void setPIDF(double p, double i, double d, double f) {
    this.kP = p;
    this.kI = i;
    this.kD = d;
    this.kF = f;
  }

  /** Sets the controller loop period in seconds (default {@code 0.02}). */
  public void setPeriod(double period) {
    this.period = period;
  }

  /** Sets the proportional gain. */
  public void setP(double p) {
    this.kP = p;
  }

  /** Sets the integral gain. */
  public void setI(double i) {
    this.kI = i;
  }

  /** Sets the derivative gain. */
  public void setD(double d) {
    this.kD = d;
  }

  /** Sets the feedforward gain. */
  public void setF(double f) {
    this.kF = f;
  }

  /** Sets the integral zone — error must be within this bound for the integrator to accumulate. */
  public void setIntegralZone(double Izone) {
    this.integralZone = Izone;
  }

  /**
   * Sets the integrator accumulator clamp bounds.
   *
   * @param max upper bound on the accumulator
   * @param min lower bound on the accumulator
   */
  public void setIntegralAccumulatorBounds(double max, double min) {
    this.maxIntegralAccumulator = max;
    this.minIntegralAccumulator = min;
  }

  /**
   * Sets the controller output clamp bounds.
   *
   * @param max maximum output value
   * @param min minimum output value
   */
  public void setOutputBounds(double max, double min) {
    this.maxOutput = max;
    this.minOutput = min;
  }

  /** Sets P, I, D and returns {@code this} for chaining. */
  public ScreamPIDConstants withPID(double p, double i, double d) {
    this.kP = p;
    this.kI = i;
    this.kD = d;
    return this;
  }

  /** Sets P, I, D, F and returns {@code this} for chaining. */
  public ScreamPIDConstants withPIDF(double p, double i, double d, double f) {
    this.kP = p;
    this.kI = i;
    this.kD = d;
    this.kF = f;
    return this;
  }

  /** Sets the loop period and returns {@code this} for chaining. */
  public ScreamPIDConstants withPeriod(double period) {
    this.period = period;
    return this;
  }

  /** Sets kP and returns {@code this} for chaining. */
  public ScreamPIDConstants withP(double p) {
    this.kP = p;
    return this;
  }

  /** Sets kI and returns {@code this} for chaining. */
  public ScreamPIDConstants withI(double i) {
    this.kI = i;
    return this;
  }

  /** Sets kD and returns {@code this} for chaining. */
  public ScreamPIDConstants withD(double d) {
    this.kD = d;
    return this;
  }

  /** Sets kF and returns {@code this} for chaining. */
  public ScreamPIDConstants withF(double f) {
    this.kF = f;
    return this;
  }

  /** Sets the integral zone and returns {@code this} for chaining. */
  public ScreamPIDConstants withIntegralZone(double Izone) {
    this.integralZone = Izone;
    return this;
  }

  /** Sets the integral accumulator bounds and returns {@code this} for chaining. */
  public ScreamPIDConstants withIntegralAccumulatorBounds(double max, double min) {
    this.maxIntegralAccumulator = max;
    this.minIntegralAccumulator = min;
    return this;
  }

  /** Sets the output bounds and returns {@code this} for chaining. */
  public ScreamPIDConstants withOutputBounds(double max, double min) {
    this.maxOutput = max;
    this.minOutput = min;
    return this;
  }

  /** Returns the loop period in seconds. */
  public double period() {
    return period;
  }

  /** Returns the proportional gain. */
  public double kP() {
    return kP;
  }

  /** Returns the integral gain. */
  public double kI() {
    return kI;
  }

  /** Returns the derivative gain. */
  public double kD() {
    return kD;
  }

  /** Returns the feedforward gain. */
  public double kF() {
    return kF;
  }

  /** Returns the integral zone threshold. */
  public double integralZone() {
    return integralZone;
  }

  /** Returns the upper bound on the integral accumulator. */
  public double maxIntegralAccumulator() {
    return maxIntegralAccumulator;
  }

  /** Returns the lower bound on the integral accumulator. */
  public double minIntegralAccumulator() {
    return minIntegralAccumulator;
  }

  /** Returns the upper output clamp. */
  public double maxOutput() {
    return maxOutput;
  }

  /** Returns the lower output clamp. */
  public double minOutput() {
    return minOutput;
  }

  /**
   * Builds a {@link Slot0Configs} combining these PID gains with the given feedforward constants.
   *
   * @param ffConstants the feedforward gains and gravity type
   */
  public Slot0Configs getSlot0Configs(FeedforwardConstants ffConstants) {
    Slot0Configs config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    config.kV = ffConstants.kV();
    config.kA = ffConstants.kA();
    config.kG = ffConstants.kG();
    config.kS = ffConstants.kS();
    config.GravityType = ffConstants.gravityType();
    return config;
  }

  /**
   * Creates a {@link ScreamPIDConstants} from the PID portion of a {@link Slot0Configs}.
   *
   * @param configs the slot configs to extract P/I/D from
   */
  public static ScreamPIDConstants fromSlot0Configs(Slot0Configs configs) {
    return new ScreamPIDConstants(configs.kP, configs.kI, configs.kD);
  }

  /**
   * Builds a {@link Slot1Configs} combining these PID gains with the given feedforward constants.
   *
   * @param ffConstants the feedforward gains
   */
  public Slot1Configs getSlot1Configs(FeedforwardConstants ffConstants) {
    Slot1Configs config = new Slot1Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    config.kV = ffConstants.kV();
    config.kA = ffConstants.kA();
    config.kG = ffConstants.kG();
    config.kS = ffConstants.kS();
    return config;
  }

  /**
   * Creates a {@link ScreamPIDConstants} from the PID portion of a {@link Slot1Configs}.
   *
   * @param configs the slot configs to extract P/I/D from
   */
  public static ScreamPIDConstants fromSlot1Configs(Slot1Configs configs) {
    return new ScreamPIDConstants(configs.kP, configs.kI, configs.kD);
  }

  /**
   * Builds a {@link Slot2Configs} combining these PID gains with the given feedforward constants.
   *
   * @param ffConstants the feedforward gains
   */
  public Slot2Configs getSlot2Configs(FeedforwardConstants ffConstants) {
    Slot2Configs config = new Slot2Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    config.kV = ffConstants.kV();
    config.kA = ffConstants.kA();
    config.kG = ffConstants.kG();
    config.kS = ffConstants.kS();
    return config;
  }

  /**
   * Creates a {@link ScreamPIDConstants} from the PID portion of a {@link Slot2Configs}.
   *
   * @param configs the slot configs to extract P/I/D from
   */
  public static ScreamPIDConstants fromSlot2Configs(Slot2Configs configs) {
    return new ScreamPIDConstants(configs.kP, configs.kI, configs.kD);
  }

  /** Creates a WPILib {@link PIDController} from these constants. */
  public PIDController getPIDController() {
    return new PIDController(kP, kI, kD, period);
  }

  /**
   * Creates a WPILib {@link PIDController} with continuous input enabled over {@code [minInput, maxInput]}.
   *
   * @param minInput lower bound of the continuous input range
   * @param maxInput upper bound of the continuous input range
   */
  public PIDController getPIDController(double minInput, double maxInput) {
    PIDController controller = new PIDController(kP, kI, kD, period);
    controller.enableContinuousInput(minInput, maxInput);
    return controller;
  }

  /**
   * Creates a WPILib {@link ProfiledPIDController} with the given motion constraints.
   *
   * @param constraints max velocity and acceleration for the motion profile
   */
  public ProfiledPIDController getProfiledPIDController(Constraints constraints) {
    return new ProfiledPIDController(kP, kI, kD, constraints, period);
  }

  /**
   * Creates a WPILib {@link ProfiledPIDController} with constraints and continuous input.
   *
   * @param constraints max velocity and acceleration for the motion profile
   * @param minInput    lower bound of the continuous input range
   * @param maxInput    upper bound of the continuous input range
   */
  public ProfiledPIDController getProfiledPIDController(
      Constraints constraints, double minInput, double maxInput) {
    ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, constraints, period);
    controller.enableContinuousInput(minInput, maxInput);
    return controller;
  }

  /** Creates a CTRE {@link PhoenixPIDController} from these constants. */
  public PhoenixPIDController getPhoenixPIDController() {
    return new PhoenixPIDController(kP, kI, kD);
  }

  /**
   * Creates a CTRE {@link PhoenixPIDController} with continuous input enabled.
   *
   * @param minInput lower bound of the continuous input range
   * @param maxInput upper bound of the continuous input range
   */
  public PhoenixPIDController getPhoenixPIDController(double minInput, double maxInput) {
    PhoenixPIDController controller = new PhoenixPIDController(kP, kI, kD);
    controller.enableContinuousInput(minInput, maxInput);
    return controller;
  }

  /** Returns {@code true} if all gains and bounds in {@code other} are identical to this instance. */
  public boolean equals(ScreamPIDConstants other) {
    return this.period == other.period
        && this.kP == other.kP
        && this.kI == other.kI
        && this.kD == other.kD
        && this.kF == other.kF
        && this.minOutput == other.minOutput
        && this.maxOutput == other.maxOutput
        && this.integralZone == other.integralZone
        && this.maxIntegralAccumulator == other.maxIntegralAccumulator
        && this.minIntegralAccumulator == other.minIntegralAccumulator;
  }

  /** Returns a deep copy of these constants. */
  public ScreamPIDConstants clone() {
    ScreamPIDConstants copy = new ScreamPIDConstants();
    copy.period = this.period;
    copy.kP = this.kP;
    copy.kI = this.kI;
    copy.kD = this.kD;
    copy.kF = this.kF;
    copy.minOutput = this.minOutput;
    copy.maxOutput = this.maxOutput;
    copy.integralZone = this.integralZone;
    copy.maxIntegralAccumulator = this.maxIntegralAccumulator;
    copy.minIntegralAccumulator = this.minIntegralAccumulator;
    return copy;
  }
}
