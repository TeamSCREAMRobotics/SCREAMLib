package com.teamscreamrobotics.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Simulates a Limelight by computing synthetic measurements from a known ground-truth pose.
 *
 * <p>All output is written directly into the {@link LimelightIOInputs} struct, bypassing
 * NetworkTables entirely. If either {@code groundTruthPoseSupplier} or {@code fieldLayout}
 * in the config is null, the inputs are left empty (no targets visible).
 */
public class LimelightIOSim implements LimelightIO {

    private static final double SIM_LATENCY_MS = 11.0;

    private final Limelight limelight;
    private final VisionConfig config;
    private final Random random = new Random();

    public LimelightIOSim(Limelight limelight, VisionConfig config) {
        this.limelight = limelight;
        this.config = config;
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        if (config.groundTruthPoseSupplier == null || config.fieldLayout == null) {
            inputs.botpose_wpiblue = new double[0];
            inputs.botpose_orb_wpiblue = new double[0];
            inputs.tv = false;
            return;
        }

        Pose2d groundTruth = config.groundTruthPoseSupplier.get();
        if (groundTruth == null) {
            inputs.tv = false;
            return;
        }

        Pose3d robotPose3d = new Pose3d(groundTruth);
        Transform3d robotToCameraTF = new Transform3d(
                limelight.robotToCamera().getTranslation(),
                limelight.robotToCamera().getRotation());
        Pose3d cameraPoseWorld = robotPose3d.transformBy(robotToCameraTF);

        double fovHalf = config.simulatedFOVDegrees / 2.0;

        List<double[]> visibleFiducials = new ArrayList<>();
        double totalDist = 0.0;
        double minDist = Double.MAX_VALUE;
        double maxDist = Double.MIN_VALUE;

        for (AprilTag tag : config.fieldLayout.getTags()) {
            Pose3d tagInCamera = tag.pose.relativeTo(cameraPoseWorld);

            double dx = tagInCamera.getX();
            double dy = tagInCamera.getY();
            double dz = tagInCamera.getZ();

            if (dx <= 0) continue;

            double horizontalAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
            if (Math.abs(horizontalAngleDeg) > fovHalf) continue;

            double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (dist > config.maxTagDistance) continue;

            visibleFiducials.add(new double[]{tag.ID, dist});
            totalDist += dist;
            minDist = Math.min(minDist, dist);
            maxDist = Math.max(maxDist, dist);
        }

        int tagCount = visibleFiducials.size();
        if (tagCount == 0) {
            inputs.tv = false;
            inputs.botpose_wpiblue = new double[0];
            inputs.botpose_orb_wpiblue = new double[0];
            return;
        }

        inputs.tv = true;
        double avgDist = totalDist / tagCount;
        double tagSpan = tagCount > 1 ? maxDist - minDist : 0.0;

        double noise = config.xyStdDevCoefficient * avgDist * avgDist / tagCount;
        double noisyX = groundTruth.getX() + random.nextGaussian() * noise;
        double noisyY = groundTruth.getY() + random.nextGaussian() * noise;
        double yawDeg = groundTruth.getRotation().getDegrees();

        // Build fiducial suffix: 7 values per tag
        int totalLen = 11 + tagCount * 7;
        double[] poseArray = new double[totalLen];
        poseArray[0] = noisyX;
        poseArray[1] = noisyY;
        poseArray[2] = 0.0;
        poseArray[3] = 0.0;
        poseArray[4] = 0.0;
        poseArray[5] = yawDeg;
        poseArray[6] = SIM_LATENCY_MS;
        poseArray[7] = tagCount;
        poseArray[8] = tagSpan;
        poseArray[9] = avgDist;
        poseArray[10] = 0.0; // avgArea — not computed in sim

        for (int i = 0; i < tagCount; i++) {
            double[] fid = visibleFiducials.get(i);
            int base = 11 + i * 7;
            poseArray[base]     = fid[0]; // id
            poseArray[base + 1] = 0.0;    // txnc
            poseArray[base + 2] = 0.0;    // tync
            poseArray[base + 3] = 0.0;    // ta
            poseArray[base + 4] = fid[1]; // distToCamera
            poseArray[base + 5] = fid[1]; // distToRobot (approx)
            // Single-tag ambiguity: small random value; multi-tag: 0
            poseArray[base + 6] = tagCount == 1 ? random.nextDouble() * 0.15 : 0.0;
        }

        double timestamp = Timer.getFPGATimestamp();
        inputs.botpose_wpiblue = poseArray;
        inputs.botpose_orb_wpiblue = poseArray;
        inputs.timestampSeconds = timestamp;
        inputs.orbTimestampSeconds = timestamp;
        inputs.tl = SIM_LATENCY_MS;
        inputs.cl = 0.0;
        inputs.rawFiducials = new double[0];
    }
}
