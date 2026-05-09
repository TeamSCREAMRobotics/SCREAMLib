package com.teamscreamrobotics.motorcontrol;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

/**
 * A telescoping arm mechanism that combines a rotational pivot with a linear extension.
 *
 * <p>Owns a {@link Pivot} and {@link Elevator} internally. The owning subsystem should call
 * {@link #updateTelemetry()} from {@code periodic()} and {@link #simIterate()} from
 * {@code simulationPeriodic()}.
 *
 * <h2>Control modes</h2>
 * <ul>
 *   <li>{@link #setGoal} — motor-space, profiled (pivot angle + extension distance)</li>
 *   <li>{@link #setGoalDirect} — motor-space, direct PID</li>
 *   <li>{@link #setEndEffectorDirect} — task-space IK, direct PID</li>
 *   <li>{@link #runToEndEffector} / {@link #runEndEffector} — task-space, straight-line profiled</li>
 * </ul>
 *
 * <h2>Gravity compensation</h2>
 * <p>When {@link TelescopingArmConfig#pivotGravityKg} is non-zero, a feedforward override
 * proportional to {@code kG * cos(angle) * (ext / maxExt)} is applied to the pivot each cycle.
 * This replaces any static {@code armFeedforward} configured on the pivot motor — do not
 * configure both.
 *
 * <h2>Assumptions</h2>
 * <p>Both the pivot and extension motors must be registered to the same {@code SubsystemBase}.
 */
public class TelescopingArm {

    private final Pivot pivot;
    private final Elevator extension;
    private final TelescopingArmConfig armConfig;
    private final TrapezoidProfile taskSpaceProfile;
    private final SubsystemBase subsystem;
    private final String logPrefix;

    public TelescopingArm(TelescopingArmConfig armConfig) {
        this.armConfig = armConfig;
        this.logPrefix = armConfig.resolveLogPrefix();
        this.subsystem = armConfig.pivotMotor.getConfig().subsystem;

        PivotConfig pivotCfg = new PivotConfig(armConfig.pivotMotor)
                .withStartingPosition(armConfig.startingAngle)
                .withLogPrefix(logPrefix + "Pivot/");
        pivotCfg.moiKgMetersSquared = armConfig.pivotMoiKgMetersSquared;
        if (armConfig.pivotMin != null && armConfig.pivotMax != null) {
            pivotCfg.withHardLimit(armConfig.pivotMin, armConfig.pivotMax);
        }
        pivot = new Pivot(pivotCfg);

        ElevatorConfig elevCfg = new ElevatorConfig(armConfig.extensionMotor)
                .withStartingHeight(armConfig.startingExtension)
                .withHardLimits(armConfig.minExtension, armConfig.maxExtension)
                .withLogPrefix(logPrefix + "Extension/");
        if (armConfig.extensionMass != null) {
            elevCfg.withMass(armConfig.extensionMass);
        }
        extension = new Elevator(elevCfg);

        taskSpaceProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(armConfig.maxTaskVelocityMps, armConfig.maxTaskAccelMps2));
    }

    // ── Accessors ──────────────────────────────────────────────────────────────

    public Pivot getPivot() { return pivot; }
    public Elevator getExtension() { return extension; }
    public Angle getAngle() { return pivot.getAngle(); }
    public Distance getExtensionDistance() { return extension.getHeight(); }

    public double getEndEffectorX() {
        return getExtensionDistance().in(Meters) * Math.cos(getAngle().in(Radians));
    }

    public double getEndEffectorY() {
        return getExtensionDistance().in(Meters) * Math.sin(getAngle().in(Radians));
    }

    // ── Commands ───────────────────────────────────────────────────────────────

    /** Commands pivot and extension continuously via Motion Magic. */
    public Command run(Angle angle, Distance ext) {
        return Commands.run(() -> setGoal(angle, ext), subsystem)
                .withName("TelescopingArm.run");
    }

    /** Commands pivot and extension to goal via Motion Magic. Ends when both are at goal. */
    public Command runTo(Angle angle, Distance ext) {
        return Commands.run(() -> setGoal(angle, ext), subsystem)
                .until(() -> atGoal(angle, ext))
                .withName("TelescopingArm.runTo");
    }

    /**
     * Moves the end effector to (x, y) in a straight line using task-space profiling.
     * Ends when the end effector is within 2 cm of the target.
     */
    public Command runToEndEffector(Distance x, Distance y) {
        TrapezoidProfile.State[] state = {null};
        double[] startXY = {0, 0};
        return Commands.sequence(
                Commands.runOnce(() -> {
                    startXY[0] = getEndEffectorX();
                    startXY[1] = getEndEffectorY();
                    state[0] = new TrapezoidProfile.State(0, 0);
                }),
                Commands.run(() -> stepTaskSpaceProfile(state, startXY, x.in(Meters), y.in(Meters)), subsystem)
                        .until(() -> atEndEffector(x, y))
        ).withName("TelescopingArm.runToEndEffector("
                + String.format("%.2f", x.in(Meters)) + "m, "
                + String.format("%.2f", y.in(Meters)) + "m)");
    }

    /**
     * Moves the end effector to (x, y) in a straight line using task-space profiling.
     * Runs until interrupted.
     */
    public Command runEndEffector(Distance x, Distance y) {
        TrapezoidProfile.State[] state = {null};
        double[] startXY = {0, 0};
        return Commands.sequence(
                Commands.runOnce(() -> {
                    startXY[0] = getEndEffectorX();
                    startXY[1] = getEndEffectorY();
                    state[0] = new TrapezoidProfile.State(0, 0);
                }),
                Commands.run(() -> stepTaskSpaceProfile(state, startXY, x.in(Meters), y.in(Meters)), subsystem)
        ).withName("TelescopingArm.runEndEffector("
                + String.format("%.2f", x.in(Meters)) + "m, "
                + String.format("%.2f", y.in(Meters)) + "m)");
    }

    // ── Control ───────────────────────────────────────────────────────────────

    /** Commands pivot and extension via Motion Magic profile with gravity compensation. */
    public void setGoal(Angle angle, Distance ext) {
        applyGravityCompensation();
        pivot.setAngleWithProfile(angle);
        extension.setHeightWithProfile(ext);
    }

    /** Commands pivot and extension via direct PID with gravity compensation. */
    public void setGoalDirect(Angle angle, Distance ext) {
        applyGravityCompensation();
        pivot.setAngle(angle);
        extension.setHeight(ext);
    }

    /**
     * IK: converts (x, y) to polar coordinates and commands pivot and extension
     * directly via PID. Extension is clamped to configured limits.
     */
    public void setEndEffectorDirect(double xMeters, double yMeters) {
        Angle angle = Radians.of(Math.atan2(yMeters, xMeters));
        double clampedExt = Math.max(armConfig.minExtension.in(Meters),
                Math.min(armConfig.maxExtension.in(Meters),
                        Math.sqrt(xMeters * xMeters + yMeters * yMeters)));
        applyGravityCompensation();
        pivot.setAngle(angle);
        extension.setHeight(Meters.of(clampedExt));
    }

    public boolean atGoal(Angle angle, Distance ext) {
        return pivot.atAngle(angle, Degrees.of(1.0)) && extension.atHeight(ext, Meters.of(0.01));
    }

    public boolean atEndEffector(Distance x, Distance y) {
        double dx = getEndEffectorX() - x.in(Meters);
        double dy = getEndEffectorY() - y.in(Meters);
        return Math.sqrt(dx * dx + dy * dy) < 0.02;
    }

    // ── Characterization & tuning ──────────────────────────────────────────────

    /**
     * Extends fully, then ramps pivot voltage up from 0 at horizontal to measure kG.
     *
     * <p>Extension must be at maximum before kG is meaningful — the full-extension
     * holding voltage is the {@link TelescopingArmConfig#pivotGravityKg} value that
     * scales gravity compensation at runtime.
     *
     * <p>Read {@code <logPrefix>Pivot/GravityChar/KgEstimate} from AKit logs and
     * plug it into {@link TelescopingArmConfig#withPivotGravity}.
     *
     * <p>WARNING: open-loop voltage. Ensure soft limits are configured and the
     * mechanism is clear of obstructions before running.
     */
    public Command gravityCharacterization() {
        return Commands.sequence(
                extension.runToWithProfile(armConfig.maxExtension),
                pivot.gravityCharacterization()
        ).withName("TelescopingArm GravityCharacterization");
    }

    /**
     * Continuously applies dashboard gain updates and commands the pivot to the
     * supplied setpoint (in rotations).
     *
     * @see SmartMechanism#tuningMode
     */
    public Command pivotTuningMode(MechanismTuner tuner, DoubleSupplier setpointRotations) {
        return pivot.tuningMode(tuner, setpointRotations);
    }

    /**
     * Continuously applies dashboard gain updates and commands the extension to the
     * supplied setpoint (in motor rotations — divide target meters by drum circumference).
     *
     * @see SmartMechanism#tuningMode
     */
    public Command extensionTuningMode(MechanismTuner tuner, DoubleSupplier setpointRotations) {
        return extension.tuningMode(tuner, setpointRotations);
    }

    // ── Simulation & telemetry ─────────────────────────────────────────────────

    public void simIterate() {
        pivot.simIterate();
        extension.simIterate();
    }

    public void updateTelemetry() {
        pivot.updateTelemetry();
        extension.updateTelemetry();

        Logger.recordOutput(logPrefix + "EndEffectorXMeters", getEndEffectorX());
        Logger.recordOutput(logPrefix + "EndEffectorYMeters", getEndEffectorY());
        Logger.recordOutput(logPrefix + "AngleDegrees", getAngle().in(Degrees));
        Logger.recordOutput(logPrefix + "ExtensionMeters", getExtensionDistance().in(Meters));
    }

    // ── Private helpers ────────────────────────────────────────────────────────

    private void applyGravityCompensation() {
        if (armConfig.pivotGravityKg == 0.0) return;
        double angleRad = getAngle().in(Radians);
        double extMeters = getExtensionDistance().in(Meters);
        double maxExt = armConfig.maxExtension.in(Meters);
        double scale = maxExt > 0 ? extMeters / maxExt : 0.0;
        pivot.getMotor().setFeedforwardOverride(armConfig.pivotGravityKg * Math.cos(angleRad) * scale);
    }

    private void stepTaskSpaceProfile(TrapezoidProfile.State[] state, double[] startXY, double tX, double tY) {
        double dx = tX - startXY[0], dy = tY - startXY[1];
        double totalDist = Math.sqrt(dx * dx + dy * dy);
        state[0] = taskSpaceProfile.calculate(0.020, state[0], new TrapezoidProfile.State(totalDist, 0));
        double frac = totalDist > 0 ? Math.min(state[0].position / totalDist, 1.0) : 1.0;
        setEndEffectorDirect(startXY[0] + frac * dx, startXY[1] + frac * dy);
    }
}
