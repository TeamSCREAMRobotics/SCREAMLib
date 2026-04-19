package com.teamscreamrobotics.math;

import com.teamscreamrobotics.data.Length;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Two-joint 2D inverse kinematics solver using the law of cosines. */
public class IKSolver {

    private final Length[] jointLengths;
    private final double minDistance;
    private final double maxDistance;

    /**
     * Creates an IK solver for a two-link arm.
     *
     * @param joint1Length length of the first link (shoulder to elbow)
     * @param joint2Length length of the second link (elbow to end-effector)
     */
    public IKSolver(Length joint1Length, Length joint2Length) {
        this.jointLengths = new Length[] { joint1Length, joint2Length };
        this.minDistance = Math.abs(joint1Length.minus(joint2Length).getMeters());
        this.maxDistance = joint1Length.plus(joint2Length).getMeters();
    }

    /**
     * Solves for the joint angles needed to reach {@code target}.
     * If the target is outside the arm's reachable range it is clamped to the nearest reachable point.
     *
     * @param target    the desired end-effector position (meters) relative to the shoulder
     * @param elbowDown {@code true} for the elbow-down configuration, {@code false} for elbow-up
     * @return {@code [theta1, theta2]} — shoulder and elbow joint angles
     */
    public Rotation2d[] solve(Translation2d target, boolean elbowDown) {
        double x = target.getX();
        double y = target.getY();
        double distanceSquared = x * x + y * y;
        double distance = Math.sqrt(distanceSquared);

        double length1 = jointLengths[0].getMeters();
        double length2 = jointLengths[1].getMeters();

        distance = MathUtil.clamp(distance, minDistance, maxDistance);
        if (distance != target.getNorm()) {
            double scale = distance / target.getNorm();
            x *= scale;
            y *= scale;
        }

        double cosTheta2 = (distanceSquared - length1 * length1 - length2 * length2) / (2 * length1 * length2);
        cosTheta2 = MathUtil.clamp(cosTheta2, -1.0, 1.0);
        double theta2 = elbowDown ? -Math.acos(cosTheta2) : Math.acos(cosTheta2);

        double k1 = length1 + length2 * Math.cos(theta2);
        double k2 = length2 * Math.sin(theta2);
        double theta1 = Math.atan2(y, x) - Math.atan2(k2, k1);

        return new Rotation2d[] {
            Rotation2d.fromRadians(theta1),
            Rotation2d.fromRadians(theta2)
        };
    }
}
