package com.teamscreamrobotics.motorcontrol;

/**
 * Initial gains and motion profile parameters for live PID tuning via {@link MechanismTuner}.
 *
 * <p>These values become the SmartDashboard defaults; override them at runtime on the dashboard
 * without redeploying code.
 */
public class TuningConfig {

    public double kP;
    public double kI;
    public double kD;
    /** Motion Magic cruise velocity (RotationsPerSecond). Ignored when {@code enableMotionMagic} is false. */
    public double maxVelocity;
    /** Motion Magic acceleration (RotationsPerSecondPerSecond). Ignored when {@code enableMotionMagic} is false. */
    public double maxAcceleration;
    /** When true, {@link MechanismTuner#update()} also pushes cruise velocity and acceleration to the motor config. */
    public boolean enableMotionMagic;

    /**
     * Full constructor — enables Motion Magic parameter tuning.
     *
     * @param maxVelocity     cruise velocity in RotationsPerSecond
     * @param maxAcceleration acceleration in RotationsPerSecondPerSecond
     */
    public TuningConfig(double kP, double kI, double kD, double maxVelocity, double maxAcceleration) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.enableMotionMagic = true;
    }

    /** PID-only constructor — Motion Magic parameters are not touched by {@link MechanismTuner#update()}. */
    public TuningConfig(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.0, 0.0);
        this.enableMotionMagic = false;
    }
}
