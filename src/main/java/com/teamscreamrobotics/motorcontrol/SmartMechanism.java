package com.teamscreamrobotics.motorcontrol;

import com.teamscreamrobotics.motorcontrol.SmartMotorControllerConfig.MechanismGearing;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

    // ── Tuning ────────────────────────────────────────────────────────────────

    /**
     * Returns a command that continuously applies dashboard gain updates and commands
     * the mechanism to the supplied setpoint (in rotations). Runs until interrupted.
     *
     * <h2>Example</h2>
     * <pre>
     *   private final MechanismTuner armTuner = new MechanismTuner(
     *       "Arm", new TuningConfig(80.0, 0.0, 2.0, 3.0, 6.0), motor);
     *
     *   public Command armTuningMode(DoubleSupplier setpointRotations) {
     *       return arm.tuningMode(armTuner, setpointRotations);
     *   }
     *
     *   // Bind in RobotContainer:
     *   // operator.povUp().whileTrue(subsystem.armTuningMode(
     *   //     () -> SmartDashboard.getNumber("Tuning/Arm/Setpoint", 0.0)));
     * </pre>
     */
    public Command tuningMode(MechanismTuner tuner, DoubleSupplier setpointSupplier) {
        return Commands.run(() -> {
            tuner.update();
            tuner.setGoalForTuning(setpointSupplier.getAsDouble());
        }, config.subsystem)
        .withName(tuner.getMechanismName() + " Tuning Mode");
    }

    // ── Characterization helpers ──────────────────────────────────────────────

    /**
     * Slowly ramps open-loop voltage until the mechanism holds steady, then invokes
     * {@code onConverged} with the holding voltage. Always resets voltage to 0 V on
     * exit, even if interrupted.
     *
     * <p>The returned command has no built-in timeout — wrap with {@code .withTimeout()}
     * at the call site if needed.
     *
     * @param maxVoltage                 upper voltage limit (safety cap)
     * @param stepVoltagePerCycle        voltage increment per 20 ms loop cycle
     * @param velocityThresholdRadPerSec mechanism is considered "holding" when
     *                                   |velocity| is below this value
     * @param consecutiveCyclesRequired  number of consecutive cycles below threshold
     *                                   required before declaring convergence
     * @param onConverged                called with the holding voltage on convergence;
     *                                   NOT called on timeout or interrupt
     */
    protected Command voltageRampCommand(
            double maxVoltage,
            double stepVoltagePerCycle,
            double velocityThresholdRadPerSec,
            int consecutiveCyclesRequired,
            Consumer<Double> onConverged) {

        double[] currentVoltage = {0.0};
        int[]    steadyCount    = {0};

        return Commands.sequence(
            Commands.runOnce(() -> {
                currentVoltage[0] = 0.0;
                steadyCount[0]    = 0;
            }),
            Commands.run(() -> {
                currentVoltage[0] = Math.min(currentVoltage[0] + stepVoltagePerCycle, maxVoltage);
                motor.setVoltage(Volts.of(currentVoltage[0]));

                double vel = Math.abs(motor.getMechanismVelocity().in(RadiansPerSecond));
                if (vel < velocityThresholdRadPerSec) {
                    steadyCount[0]++;
                } else {
                    steadyCount[0] = 0;
                }
            }, config.subsystem)
            .until(() -> steadyCount[0] >= consecutiveCyclesRequired)
        )
        .andThen(Commands.runOnce(() -> onConverged.accept(currentVoltage[0])))
        .finallyDo(() -> motor.setVoltage(Volts.of(0.0)));
    }

    /**
     * Applies {@code startVoltage}, then slowly ramps voltage down until the mechanism
     * holds steady. Avoids the false-positive that {@link #voltageRampCommand} produces
     * when the mechanism starts at rest (e.g. an elevator sitting on its lower hard stop).
     *
     * <p>The returned command has no built-in timeout — wrap with {@code .withTimeout()}
     * at the call site if needed.
     *
     * @param startVoltage               initial voltage, should be above kG to ensure
     *                                   the mechanism lifts off before ramping down
     * @param stepVoltagePerCycle        voltage decrement per 20 ms loop cycle
     * @param velocityThresholdRadPerSec mechanism is considered "holding" when
     *                                   |velocity| is below this value
     * @param consecutiveCyclesRequired  number of consecutive cycles below threshold
     *                                   required before declaring convergence
     * @param onConverged                called with the holding voltage on convergence;
     *                                   NOT called on timeout or interrupt
     */
    protected Command voltageRampDownCommand(
            double startVoltage,
            double stepVoltagePerCycle,
            double velocityThresholdRadPerSec,
            int consecutiveCyclesRequired,
            Consumer<Double> onConverged) {

        double[] currentVoltage = {0.0};
        int[]    steadyCount    = {0};

        return Commands.sequence(
            Commands.runOnce(() -> {
                currentVoltage[0] = startVoltage;
                steadyCount[0]    = 0;
            }),
            Commands.run(() -> {
                currentVoltage[0] = Math.max(currentVoltage[0] - stepVoltagePerCycle, 0.0);
                motor.setVoltage(Volts.of(currentVoltage[0]));

                double vel = Math.abs(motor.getMechanismVelocity().in(RadiansPerSecond));
                if (vel < velocityThresholdRadPerSec) {
                    steadyCount[0]++;
                } else {
                    steadyCount[0] = 0;
                }
            }, config.subsystem)
            .until(() -> steadyCount[0] >= consecutiveCyclesRequired)
        )
        .andThen(Commands.runOnce(() -> onConverged.accept(currentVoltage[0])))
        .finallyDo(() -> motor.setVoltage(Volts.of(0.0)));
    }
}
