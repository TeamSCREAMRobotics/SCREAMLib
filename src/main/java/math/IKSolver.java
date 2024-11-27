package math;

import data.Length;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class IKSolver {

    private final Length[] jointLengths;
    private final double minDistance;
    private final double maxDistance;

    public IKSolver(Length joint1Length, Length joint2Length) {
        this.jointLengths = new Length[] { joint1Length, joint2Length };
        this.minDistance = Math.abs(joint1Length.minus(joint2Length).getMeters());
        this.maxDistance = joint1Length.plus(joint2Length).getMeters();
    }

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
