package com.teamscreamrobotics.motorcontrol;

import org.littletonrobotics.junction.Logger;

import com.teamscreamrobotics.dashboard.LoggedTunableNumber;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Owns a set of {@link LoggedTunableNumber}s for one mechanism and pushes gain
 * updates to the motor controller when any value changes on the dashboard.
 *
 * <p>Attach to any mechanism by calling {@link #update()} from the owning subsystem's
 * {@code periodic()}, or use {@link SmartMechanism#tuningMode(MechanismTuner, java.util.function.DoubleSupplier)}
 * to bind it to a command.
 *
 * <h2>Example</h2>
 * <pre>
 *   private final MechanismTuner armTuner = new MechanismTuner(
 *       "Arm", new TuningConfig(80.0, 0.0, 2.0, 3.0, 6.0), motor);
 *
 *   public Command armTuningMode(DoubleSupplier setpointRotations) {
 *       return arm.tuningMode(armTuner, setpointRotations);
 *   }
 * </pre>
 *
 * Dashboard keys are published under {@code Tuning/<mechanismName>/kP} etc.
 * Setpoints passed to {@link #setGoalForTuning} are in rotations (TalonFX native units).
 */
public class MechanismTuner {

    private final String mechanismName;
    private final TuningConfig tuningConfig;
    private final TalonFXWrapper motor;

    public final LoggedTunableNumber kP;
    public final LoggedTunableNumber kI;
    public final LoggedTunableNumber kD;
    public final LoggedTunableNumber maxVelocity;
    public final LoggedTunableNumber maxAcceleration;

    public MechanismTuner(String mechanismName, TuningConfig initialConfig, TalonFXWrapper motor) {
        this.mechanismName = mechanismName;
        this.tuningConfig = initialConfig;
        this.motor = motor;

        String base = "Tuning/" + mechanismName + "/";
        kP             = new LoggedTunableNumber(base + "kP",             initialConfig.kP);
        kI             = new LoggedTunableNumber(base + "kI",             initialConfig.kI);
        kD             = new LoggedTunableNumber(base + "kD",             initialConfig.kD);
        maxVelocity    = new LoggedTunableNumber(base + "MaxVelocity",    initialConfig.maxVelocity);
        maxAcceleration = new LoggedTunableNumber(base + "MaxAcceleration", initialConfig.maxAcceleration);
    }

    /**
     * Call from periodic(). Checks all five tunable numbers; if any changed, pushes
     * updated gains to the motor controller. Always logs current values to AKit.
     */
    public void update() {
        // Use | (not ||) so every hasChanged() runs and updates its cached value.
        boolean changed = kP.hasChanged() | kI.hasChanged() | kD.hasChanged()
                        | maxVelocity.hasChanged() | maxAcceleration.hasChanged();

        if (changed) {
            motor.setClosedLoopGains(kP.get(), kI.get(), kD.get());
            if (tuningConfig.enableMotionMagic) {
                motor.setMotionMagicConstraints(maxVelocity.get(), maxAcceleration.get());
            }
        }

        String base = "Tuning/" + mechanismName + "/";
        Logger.recordOutput(base + "kP",             kP.get());
        Logger.recordOutput(base + "kI",             kI.get());
        Logger.recordOutput(base + "kD",             kD.get());
        Logger.recordOutput(base + "MaxVelocity",    maxVelocity.get());
        Logger.recordOutput(base + "MaxAcceleration", maxAcceleration.get());
        Logger.recordOutput(base + "UpdateApplied",  changed);
    }

    /**
     * Sends {@code rotations} as a position setpoint to the motor for testing current gains.
     *
     * @param rotations target position in rotations (TalonFX native units)
     */
    public void setGoalForTuning(double rotations) {
        if (tuningConfig.enableMotionMagic) {
            motor.setPositionProfiled(Rotation2d.fromRotations(rotations));
        } else {
            motor.setPosition(Rotation2d.fromRotations(rotations));
        }
    }

    public String getMechanismName() {
        return mechanismName;
    }
}
