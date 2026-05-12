package com.teamscreamrobotics.motorcontrol;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

/**
 * Abstract base class for all SCREAMLib mechanisms.
 *
 * <p>Mechanisms are NOT subsystems — they are owned and ticked by a subsystem.
 * The owning subsystem should call {@link #updateTelemetry()} from {@code periodic()}
 * and {@link #simIterate()} from {@code simulationPeriodic()}.
 *
 * <h2>AdvantageKit log structure</h2>
 * Each mechanism produces two categories of log entries:
 * <ul>
 *   <li><b>Replayed inputs</b> (hardware reads) — written via
 *       {@code Logger.processInputs(logPrefix + "Motor", motor.getInputs())}
 *       and therefore injectable during AKit replay.
 *   </li>
 *   <li><b>Outputs</b> (derived values) — computed from inputs, so they replay
 *       automatically once inputs replay.
 *   </li>
 * </ul>
 */
public abstract class SmartMechanism {

    protected final TalonFXWrapper motor;
    protected final String logPrefix;

    public SmartMechanism(TalonFXWrapper motor, String logPrefix) {
        this.motor = motor;
        this.logPrefix = logPrefix;
    }

    protected SubsystemBase getSubsystem() {
        return motor.getRuntimeInfo().subsystem();
    }

    /**
     * Refreshes hardware inputs and calls {@code Logger.processInputs}, satisfying
     * the AKit IO contract for this motor. Must be the first call in {@link #updateTelemetry()}.
     */
    protected void processInputs() {
        motor.updateInputs(motor.getInputs());
        Logger.processInputs(logPrefix + "Motor", motor.getInputs());
    }

    public abstract void simIterate();

    public abstract void updateTelemetry();

    public TalonFXWrapper getMotor() {
        return motor;
    }

    /**
     * Computes the overall gear reduction from a series of stage ratios.
     * Example: {@code gearing(10, 6)} returns {@code 60.0} for a 60:1 overall reduction.
     */
    public static double gearing(double... stages) {
        double product = 1.0;
        for (double s : stages) product *= s;
        return product;
    }

    // ── Tuning ────────────────────────────────────────────────────────────────

    /**
     * Returns a command that continuously applies dashboard gain updates and commands
     * the mechanism to the supplied setpoint (in rotations). Runs until interrupted.
     */
    public Command tuningMode(MechanismTuner tuner, DoubleSupplier setpointSupplier) {
        return Commands.run(() -> {
            tuner.update();
            tuner.setGoalForTuning(setpointSupplier.getAsDouble());
        }, getSubsystem())
        .withName(tuner.getMechanismName() + " Tuning Mode");
    }

    // ── Characterization helpers ──────────────────────────────────────────────

    /**
     * Slowly ramps open-loop voltage until the mechanism holds steady, then invokes
     * {@code onConverged} with the holding voltage. Always resets voltage to 0 V on exit.
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
            Commands.runOnce(() -> { currentVoltage[0] = 0.0; steadyCount[0] = 0; }),
            Commands.run(() -> {
                currentVoltage[0] = Math.min(currentVoltage[0] + stepVoltagePerCycle, maxVoltage);
                motor.setVoltage(Volts.of(currentVoltage[0]));
                double vel = Math.abs(motor.getMechanismVelocity().in(RadiansPerSecond));
                steadyCount[0] = vel < velocityThresholdRadPerSec ? steadyCount[0] + 1 : 0;
            }, getSubsystem())
            .until(() -> steadyCount[0] >= consecutiveCyclesRequired)
        )
        .andThen(Commands.runOnce(() -> onConverged.accept(currentVoltage[0])))
        .finallyDo(() -> motor.setVoltage(Volts.of(0.0)));
    }

    /**
     * Applies {@code startVoltage}, then slowly ramps voltage down until the mechanism holds steady.
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
            Commands.runOnce(() -> { currentVoltage[0] = startVoltage; steadyCount[0] = 0; }),
            Commands.run(() -> {
                currentVoltage[0] = Math.max(currentVoltage[0] - stepVoltagePerCycle, 0.0);
                motor.setVoltage(Volts.of(currentVoltage[0]));
                double vel = Math.abs(motor.getMechanismVelocity().in(RadiansPerSecond));
                steadyCount[0] = vel < velocityThresholdRadPerSec ? steadyCount[0] + 1 : 0;
            }, getSubsystem())
            .until(() -> steadyCount[0] >= consecutiveCyclesRequired)
        )
        .andThen(Commands.runOnce(() -> onConverged.accept(currentVoltage[0])))
        .finallyDo(() -> motor.setVoltage(Volts.of(0.0)));
    }
}
