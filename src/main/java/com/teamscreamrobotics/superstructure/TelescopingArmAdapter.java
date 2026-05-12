package com.teamscreamrobotics.superstructure;

import com.teamscreamrobotics.motorcontrol.TelescopingArm;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Meters;

/**
 * Bridges {@link TelescopingArm} to the superstructure graph by exposing its pivot and
 * extension as separate {@link MechanismAdapter} instances.
 *
 * <p>Usage in a superstructure state:
 * <pre>
 *   TelescopingArmAdapter adapter = new TelescopingArmAdapter(telescopingArm, "Arm");
 *
 *   // Angle + extension directly:
 *   .set(adapter.getPivotAdapter(), Rotation2d.fromDegrees(45))
 *   .set(adapter.getExtensionAdapter(), Meters.of(0.5))
 *
 *   // Or via the IK helper:
 *   double[] pos = TelescopingArmAdapter.positionFromEndEffector(Meters.of(0.5), Meters.of(0.3));
 *   .set(adapter.getPivotAdapter(), pos[0])
 *   .set(adapter.getExtensionAdapter(), pos[1])
 * </pre>
 *
 * <p>For task-space profiled moves use {@link TelescopingArm#runToEndEffector} and
 * {@link TelescopingArm#runEndEffector} directly, accessed via {@link #getArm()}.
 */
public class TelescopingArmAdapter {

    private final TelescopingArm arm;
    private final PivotAdapter pivotAdapter;
    private final ElevatorAdapter extensionAdapter;

    public TelescopingArmAdapter(TelescopingArm arm) {
        this(arm, "TelescopingArm");
    }

    public TelescopingArmAdapter(TelescopingArm arm, String name) {
        this.arm = arm;
        this.pivotAdapter = new PivotAdapter(name + "/Pivot", arm.getPivot());
        this.extensionAdapter = new ElevatorAdapter(name + "/Extension", arm.getExtension());
    }

    public PivotAdapter getPivotAdapter() { return pivotAdapter; }
    public ElevatorAdapter getExtensionAdapter() { return extensionAdapter; }
    public TelescopingArm getArm() { return arm; }

    /**
     * Converts end-effector Cartesian coordinates to {@code {angleDegrees, extensionMeters}}
     * for use with {@link #getPivotAdapter()} and {@link #getExtensionAdapter()}.
     */
    public static double[] positionFromEndEffector(Distance x, Distance y) {
        double xM = x.in(Meters), yM = y.in(Meters);
        return new double[]{
                Math.toDegrees(Math.atan2(yM, xM)),
                Math.sqrt(xM * xM + yM * yM)
        };
    }
}
