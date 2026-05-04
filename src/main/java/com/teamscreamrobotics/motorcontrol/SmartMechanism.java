package com.teamscreamrobotics.motorcontrol;

import com.teamscreamrobotics.motorcontrol.SmartMotorControllerConfig.MechanismGearing;

import org.littletonrobotics.junction.Logger;

/**
 * Abstract base class for all SCREAMLib mechanisms.
 *
 * <p>Mechanisms are NOT subsystems -- they are owned and ticked by a subsystem.
 * The owning subsystem should call {@link #updateTelemetry()} from {@code periodic()}
 * and {@link #simIterate()} from {@code simulationPeriodic()}.
 *
 * <h2>AdvantageKit log structure</h2>
 * Each mechanism produces two categories of log entries:
 * <ul>
 *   <li><b>Replayed inputs</b> (hardware reads) -- written via
 *       {@code Logger.processInputs(logPrefix + "Motor", motor.getInputs())}
 *       and therefore injectable during AKit replay:
 *       <pre>
 *       &lt;logPrefix&gt;Motor/PositionRad
 *       &lt;logPrefix&gt;Motor/VelocityRadPerSec
 *       &lt;logPrefix&gt;Motor/AppliedVolts
 *       &lt;logPrefix&gt;Motor/SupplyCurrentAmps
 *       &lt;logPrefix&gt;Motor/StatorCurrentAmps
 *       &lt;logPrefix&gt;Motor/TempCelsius
 *       &lt;logPrefix&gt;Motor/Connected
 *       </pre>
 *   </li>
 *   <li><b>Outputs</b> (derived values) -- written via {@code Logger.recordOutput}
 *       and computed from inputs, so they replay automatically once inputs replay:
 *       <pre>
 *       &lt;logPrefix&gt;AngleDegrees  (or HeightMeters, VelocityRPM, etc.)
 *       &lt;logPrefix&gt;SetpointDegrees
 *       &lt;logPrefix&gt;AtGoal
 *       &lt;logPrefix&gt;ActiveCommand
 *       </pre>
 *   </li>
 * </ul>
 *
 * <p>Example for an {@code Arm} owned by {@code ShooterSubsystem}:
 * <pre>
 *   Mechanisms/Arm/ShooterSubsystem/Motor/PositionRad    &lt;-- replayed input
 *   Mechanisms/Arm/ShooterSubsystem/Motor/VelocityRadPerSec
 *   Mechanisms/Arm/ShooterSubsystem/Motor/AppliedVolts
 *   Mechanisms/Arm/ShooterSubsystem/Motor/SupplyCurrentAmps
 *   Mechanisms/Arm/ShooterSubsystem/Motor/StatorCurrentAmps
 *   Mechanisms/Arm/ShooterSubsystem/Motor/TempCelsius
 *   Mechanisms/Arm/ShooterSubsystem/Motor/Connected
 *   Mechanisms/Arm/ShooterSubsystem/AngleDegrees         &lt;-- output
 *   Mechanisms/Arm/ShooterSubsystem/SetpointDegrees
 *   Mechanisms/Arm/ShooterSubsystem/AtAngle
 *   Mechanisms/Arm/ShooterSubsystem/ActiveCommand
 * </pre>
 */
public abstract class SmartMechanism {

    protected final SmartMotorController motor;
    protected final SmartMotorControllerConfig config;
    protected final String logPrefix;

    public SmartMechanism(SmartMotorController motor, String logPrefix) {
        this.motor = motor;
        this.config = motor.getConfig();
        this.logPrefix = logPrefix;
    }

    /**
     * Refreshes hardware inputs and calls {@code Logger.processInputs}, satisfying
     * the AKit IO contract for this motor. Must be the first call in
     * {@link #updateTelemetry()}.
     */
    protected void processInputs() {
        motor.updateInputs(motor.getInputs());
        Logger.processInputs(logPrefix + "Motor", motor.getInputs());
    }

    public abstract void simIterate();

    public abstract void updateTelemetry();

    public SmartMotorController getMotor() {
        return motor;
    }

    public static MechanismGearing gearing(double... stages) {
        return MechanismGearing.fromReductionStages(stages);
    }

    public static MechanismGearing gearing(MechanismGearing g) {
        return g;
    }
}
