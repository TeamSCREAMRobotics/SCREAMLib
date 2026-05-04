package com.teamscreamrobotics.motorcontrol.example;

import com.ctre.phoenix6.hardware.TalonFX;

import com.teamscreamrobotics.motorcontrol.Arm;
import com.teamscreamrobotics.motorcontrol.TalonFXWrapper;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

/**
 * Example arm subsystem showing the complete SCREAMLib usage pattern with full
 * AdvantageKit replay support.
 *
 * <h2>Three-mode transparency</h2>
 * The same constructor runs identically in all three runtime modes -- no
 * subclassing or conditional instantiation is needed:
 *
 * <pre>
 *   // RobotContainer -- identical in all three modes:
 *   ExampleSubsystem arm = new ExampleSubsystem();
 *
 *   // Real robot:  TalonFXWrapper reads CAN signals; processInputs records them.
 *   // Simulation:  TalonFXWrapper reads from DCMotorSim/SingleJointedArmSim;
 *   //              Logger.hasReplaySource() == false so physics runs normally.
 *   // Replay:      Logger.hasReplaySource() == true, so:
 *   //                - updateInputs() is a no-op (no CAN, no physics)
 *   //                - simIterate() is a no-op (physics skipped)
 *   //                - Logger.processInputs() injects all sensor values from
 *   //                  the log file before any mechanism code reads them.
 * </pre>
 *
 * <h2>AdvantageScope log paths</h2>
 *
 * <pre>
 * INPUTS (replayable -- AKit injects these from the log during replay):
 *   Mechanisms/Arm/ExampleSubsystem/Motor/PositionRad
 *   Mechanisms/Arm/ExampleSubsystem/Motor/VelocityRadPerSec
 *   Mechanisms/Arm/ExampleSubsystem/Motor/AppliedVolts
 *   Mechanisms/Arm/ExampleSubsystem/Motor/SupplyCurrentAmps
 *   Mechanisms/Arm/ExampleSubsystem/Motor/StatorCurrentAmps
 *   Mechanisms/Arm/ExampleSubsystem/Motor/TempCelsius
 *   Mechanisms/Arm/ExampleSubsystem/Motor/Connected
 *
 * OUTPUTS (derived from inputs -- replay-correct automatically):
 *   Mechanisms/Arm/ExampleSubsystem/AngleDegrees
 *   Mechanisms/Arm/ExampleSubsystem/SetpointDegrees
 *   Mechanisms/Arm/ExampleSubsystem/AtAngle
 *   Mechanisms/Arm/ExampleSubsystem/ActiveCommand
 * </pre>
 */
public class ExampleSubsystem extends SubsystemBase {

    public enum Goal {
        STOW(Degrees.of(0.0)),
        INTAKE(Degrees.of(-30.0)),
        SCORE(Degrees.of(85.0));

        public final Angle angle;

        Goal(Angle angle) {
            this.angle = angle;
        }
    }

    private final Arm arm;

    public ExampleSubsystem() {
        TalonFX talonFX = new TalonFX(Constants.Arm.CAN_ID);

        TalonFXWrapper motor = new TalonFXWrapper(
                talonFX,
                Constants.Arm.MOTOR_MODEL,
                Constants.Arm.motorConfig(this));

        arm = new Arm(Constants.Arm.armConfig(motor));
    }

    // -------------------------------------------------------------------------
    // Subsystem lifecycle
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        arm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        arm.simIterate();
    }

    // -------------------------------------------------------------------------
    // Command factories
    // -------------------------------------------------------------------------

    /** Continuously drives the arm to the intake position. Never ends on its own. */
    public Command moveToIntake() {
        return arm.run(Goal.INTAKE.angle)
                .withName("MoveToIntake");
    }

    /** Drives the arm to the score position and ends when it arrives. */
    public Command moveToScore() {
        return arm.runTo(Goal.SCORE.angle)
                .withName("MoveToScore");
    }

    /** Continuously drives the arm to the stow position. Never ends on its own. */
    public Command moveToStow() {
        return arm.run(Goal.STOW.angle)
                .withName("MoveToStow");
    }

    // -------------------------------------------------------------------------
    // State accessors (read from inputs -- safe during replay)
    // -------------------------------------------------------------------------

    public Angle getAngle() {
        return arm.getAngle();
    }

    public boolean atGoal(Goal goal) {
        return arm.atAngle(goal.angle, Degrees.of(1.5));
    }
}
