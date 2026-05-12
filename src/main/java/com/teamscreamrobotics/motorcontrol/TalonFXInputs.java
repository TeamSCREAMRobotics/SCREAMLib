package com.teamscreamrobotics.motorcontrol;

import org.littletonrobotics.junction.AutoLog;

/** Hardware inputs read from a TalonFX each cycle and replayed by AdvantageKit. */
@AutoLog
public class TalonFXInputs {
    // Master motor
    /** Mechanism position in radians (latency-compensated). */
    public double positionRad = 0.0;
    /** Mechanism velocity in radians per second. */
    public double velocityRadPerSec = 0.0;
    /** Motor output voltage in volts. */
    public double appliedVolts = 0.0;
    /** Supply (battery-side) current in amps. */
    public double supplyCurrentAmps = 0.0;
    /** Stator (motor-side) current in amps. */
    public double statorCurrentAmps = 0.0;
    /** Motor temperature in Celsius. */
    public double tempCelsius = 0.0;
    /** True when all master motor status signals are fresh. */
    public boolean connected = false;

    // Per-follower arrays — index matches the followers[] order passed to TalonFXWrapper.
    // Logged as inputs (replayable) to support current-based game piece detection that
    // reads follower currents independently during replay.
    /** Supply current for each follower motor (amps), indexed by follower order. */
    public double[] followerSupplyCurrentAmps = new double[0];
    /** Stator current for each follower motor (amps), indexed by follower order. */
    public double[] followerStatorCurrentAmps = new double[0];
    /** Temperature for each follower motor (Celsius), indexed by follower order. */
    public double[] followerTempCelsius = new double[0];
    /** True for each follower whose status signals are fresh. */
    public boolean[] followerConnected = new boolean[0];

    // CANcoder absolute position (0.0 / false when no CANcoder is configured)
    /** Absolute CANcoder position in radians. 0.0 when no CANcoder is configured. */
    public double cancoderPositionRad = 0.0;
    /** True when the CANcoder status signal is fresh. False when no CANcoder is configured. */
    public boolean cancoderConnected = false;
}
