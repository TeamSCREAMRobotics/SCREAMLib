package com.teamscreamrobotics.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.RobotBase;

import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Multi-camera vision system built on Limelight hardware.
 *
 * <h2>Usage</h2>
 * <pre>
 *   LimelightVision vision = new LimelightVision(
 *       visionConfig,
 *       gyro::getRotation2d,
 *       gyro::getRate,
 *       drive::getChassisSpeeds,
 *       poseEstimator::setVisionMeasurementStdDevs,
 *       poseEstimator::addVisionMeasurement);
 *
 *   // In drivetrain periodic():
 *   vision.update(PoseEstimationStrategy.MEGATAG_2);
 * </pre>
 *
 * <p>The vision system manages stddev tuning internally; callers never need to call
 * {@code setVisionMeasurementStdDevs} manually.
 */
public class LimelightVision {

    /** Outcome of one camera's pose estimate for a single cycle. */
    public enum VisionType {
        ACCEPTED,
        REJECTED_INVALID,
        REJECTED_AMBIGUITY,
        REJECTED_MOVEMENT
    }

    private final VisionConfig config;
    private final Supplier<Rotation2d> headingSupplier;
    private final Supplier<Double> yawRateSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    private final Consumer<Matrix<N3, N1>> stdDevSetter;
    private final BiConsumer<Pose2d, Double> poseConsumer;

    private final LimelightIO[] ios;
    private final LimelightIOInputsAutoLogged[] inputs;
    private final Map<String, Integer> limelightIndex = new HashMap<>();

    private final Map<String, VisionMeasurement> latestMeasurements = new HashMap<>();
    private final Map<String, Pose2d> rawPoses = new HashMap<>();
    private final Map<String, Double> tagDistances = new HashMap<>();

    // ── Constructors ──────────────────────────────────────────────────────────

    /**
     * Primary constructor. IO layer is auto-selected: {@link LimelightIOSim} in simulation,
     * {@link LimelightIOReal} on real hardware.
     *
     * @param stdDevSetter wraps {@code SwerveDrivePoseEstimator::setVisionMeasurementStdDevs}
     * @param poseConsumer wraps {@code SwerveDrivePoseEstimator::addVisionMeasurement}
     */
    public LimelightVision(
            VisionConfig config,
            Supplier<Rotation2d> headingSupplier,
            Supplier<Double> yawRateSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            Consumer<Matrix<N3, N1>> stdDevSetter,
            BiConsumer<Pose2d, Double> poseConsumer) {
        this(config, headingSupplier, yawRateSupplier, chassisSpeedsSupplier,
                stdDevSetter, poseConsumer, null);
    }

    /**
     * Overload for injecting custom IO implementations (testing / advanced usage).
     * If {@code ios} is null, auto-constructs real or sim IO per camera.
     */
    public LimelightVision(
            VisionConfig config,
            Supplier<Rotation2d> headingSupplier,
            Supplier<Double> yawRateSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            Consumer<Matrix<N3, N1>> stdDevSetter,
            BiConsumer<Pose2d, Double> poseConsumer,
            LimelightIO[] ios) {
        this.config = config;
        this.headingSupplier = headingSupplier;
        this.yawRateSupplier = yawRateSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        this.stdDevSetter = stdDevSetter;
        this.poseConsumer = poseConsumer;

        int count = config.limelights.length;
        this.inputs = new LimelightIOInputsAutoLogged[count];
        this.ios = new LimelightIO[count];

        for (int i = 0; i < count; i++) {
            Limelight ll = config.limelights[i];
            limelightIndex.put(ll.name(), i);
            inputs[i] = new LimelightIOInputsAutoLogged();
            if (ios != null) {
                this.ios[i] = ios[i];
            } else if (RobotBase.isSimulation()) {
                this.ios[i] = new LimelightIOSim(ll, config);
            } else {
                this.ios[i] = new LimelightIOReal(ll);
            }
        }
    }

    // ── Core update ───────────────────────────────────────────────────────────

    /**
     * Processes all cameras for one robot cycle. Call from the drivetrain's {@code periodic()}.
     *
     * @param strategy MT1 (standard solve) or MT2 (gyro-locked heading)
     */
    public void update(PoseEstimationStrategy strategy) {
        double yawDeg    = headingSupplier.get().getDegrees();
        double yawRateDps = yawRateSupplier.get();

        ChassisSpeeds speeds = chassisSpeedsSupplier.get();
        double linearVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        for (int i = 0; i < config.limelights.length; i++) {
            Limelight ll = config.limelights[i];
            LimelightIOInputsAutoLogged inp = inputs[i];

            // 1. Refresh inputs (real: reads NT; sim: computes synthetic data).
            ios[i].updateInputs(inp);
            Logger.processInputs("Vision/" + ll.name(), inp);

            // 2. Set robot orientation for MegaTag2 heading assistance (real hardware only).
            if (!RobotBase.isSimulation()) {
                LimelightHelpers.SetRobotOrientation(
                        ll.name(), yawDeg, yawRateDps, 0, 0, 0, 0);
            }

            // 3. Parse PoseEstimate directly from already-logged inputs.
            LimelightHelpers.PoseEstimate estimate = parsePoseEstimate(inp, strategy);

            // 4. Run rejection pipeline.
            double ambiguity = (estimate.tagCount == 1 && estimate.rawFiducials.length > 0)
                    ? estimate.rawFiducials[0].ambiguity : 0.0;

            VisionType visionType = classify(estimate, ambiguity, yawRateDps, linearVelocity);

            // 5. Store raw pose for debugging regardless of acceptance.
            rawPoses.put(ll.name(), estimate.pose);
            tagDistances.put(ll.name(), estimate.avgTagDist);

            // 6. If accepted, compute stddevs and forward to pose estimator.
            VisionMeasurement measurement = null;
            if (visionType == VisionType.ACCEPTED) {
                double avgDist = estimate.avgTagDist;
                int tagCount = estimate.tagCount;
                double xyStdDev    = config.xyStdDevCoefficient    * avgDist * avgDist / tagCount;
                double thetaStdDev = config.thetaStdDevCoefficient * avgDist * avgDist / tagCount;
                Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
                measurement = new VisionMeasurement(estimate.pose, estimate.timestampSeconds, stdDevs);
                latestMeasurements.put(ll.name(), measurement);

                stdDevSetter.accept(stdDevs);
                poseConsumer.accept(estimate.pose, estimate.timestampSeconds);
            }

            // 7. AKit output logging.
            boolean accepted = visionType == VisionType.ACCEPTED;
            Logger.recordOutput("Vision/" + ll.name() + "/Pose",
                    accepted ? estimate.pose : new Pose2d());
            Logger.recordOutput("Vision/" + ll.name() + "/RawPose",         estimate.pose);
            Logger.recordOutput("Vision/" + ll.name() + "/VisionType",      visionType.name());
            Logger.recordOutput("Vision/" + ll.name() + "/TagCount",        estimate.tagCount);
            Logger.recordOutput("Vision/" + ll.name() + "/AvgTagDistance",  estimate.avgTagDist);
            Logger.recordOutput("Vision/" + ll.name() + "/Ambiguity",       ambiguity);
            Logger.recordOutput("Vision/" + ll.name() + "/Latency",         estimate.latency);
            Logger.recordOutput("Vision/" + ll.name() + "/Simulated",       RobotBase.isSimulation());

            if (accepted && measurement != null) {
                double xyStdDev    = measurement.stdDevs().get(0, 0);
                double thetaStdDev = measurement.stdDevs().get(2, 0);
                Logger.recordOutput("Vision/" + ll.name() + "/XYStdDev",    xyStdDev);
                Logger.recordOutput("Vision/" + ll.name() + "/ThetaStdDev", thetaStdDev);
            } else {
                Logger.recordOutput("Vision/" + ll.name() + "/XYStdDev",    0.0);
                Logger.recordOutput("Vision/" + ll.name() + "/ThetaStdDev", 0.0);
            }
        }
    }

    // ── Public accessors ──────────────────────────────────────────────────────

    /** Returns the most recently accepted measurement for {@code limelight}, if any. */
    public Optional<VisionMeasurement> getLatestMeasurement(Limelight limelight) {
        return Optional.ofNullable(latestMeasurements.get(limelight.name()));
    }

    /** Returns true if the limelight currently reports a valid target. */
    public boolean hasTarget(Limelight limelight) {
        Integer idx = limelightIndex.get(limelight.name());
        return idx != null && inputs[idx].tv;
    }

    /** Returns the most recently computed average tag distance for this camera, in meters. */
    public double getTagDistance(Limelight limelight) {
        return tagDistances.getOrDefault(limelight.name(), 0.0);
    }

    /** Returns the most recently parsed (unfiltered, pre-rejection) pose for debugging. */
    public Pose2d getRawPose(Limelight limelight) {
        return rawPoses.getOrDefault(limelight.name(), new Pose2d());
    }

    /** Sets a specific camera to the given pipeline index. */
    public void setPipeline(Limelight limelight, int index) {
        LimelightHelpers.setPipelineIndex(limelight.name(), index);
    }

    /** Sets all cameras to the same pipeline index. */
    public void setAllPipelines(int index) {
        for (Limelight ll : config.limelights) {
            LimelightHelpers.setPipelineIndex(ll.name(), index);
        }
    }

    // ── Camera configuration ──────────────────────────────────────────────────

    /**
     * Sets a 3D offset point on a specific camera so that targeting solutions
     * aim at a point offset from the tag centre (e.g. a goal aperture above the tag).
     * Units are meters; axes match the tag's local coordinate frame.
     */
    public void setFiducial3DOffset(Limelight limelight, double x, double y, double z) {
        LimelightHelpers.setFiducial3DOffset(limelight.name(), x, y, z);
    }

    // ── Throttling ────────────────────────────────────────────────────────────

    /**
     * Sets the frame-skip throttle for one camera. {@code 0} = full rate;
     * {@code N} = process one frame then skip N, effectively running at 1/(N+1) rate.
     * Set to 100–200 while disabled to reduce heat.
     */
    public void setThrottle(Limelight limelight, int throttle) {
        LimelightHelpers.SetThrottle(limelight.name(), throttle);
    }

    /** Sets the same throttle value on every camera. */
    public void setThrottleAll(int throttle) {
        for (Limelight ll : config.limelights) LimelightHelpers.SetThrottle(ll.name(), throttle);
    }

    // ── Fiducial ID filter ────────────────────────────────────────────────────

    /**
     * Restricts one camera to only report tags whose IDs are in {@code ids}.
     * Tags not in the list are ignored for pose estimation.
     */
    public void setFiducialIDFilter(Limelight limelight, int... ids) {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelight.name(), ids);
    }

    /** Applies the same tag ID filter to every camera. */
    public void setFiducialIDFilterAll(int... ids) {
        for (Limelight ll : config.limelights)
            LimelightHelpers.SetFiducialIDFiltersOverride(ll.name(), ids);
    }

    /** Clears the ID filter on one camera so all tag IDs are accepted again. */
    public void clearFiducialIDFilter(Limelight limelight) {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelight.name(), new int[0]);
    }

    /** Clears the ID filter on every camera. */
    public void clearFiducialIDFilterAll() {
        for (Limelight ll : config.limelights)
            LimelightHelpers.SetFiducialIDFiltersOverride(ll.name(), new int[0]);
    }

    // ── Downscaling ───────────────────────────────────────────────────────────

    /**
     * Overrides the AprilTag detection downscale factor for one camera.
     * Valid values: {@code 0} (pipeline default), {@code 1.0}, {@code 1.5},
     * {@code 2.0}, {@code 3.0}, {@code 4.0}. Higher values improve CPU performance
     * but reduce detection range.
     */
    public void setDownscaling(Limelight limelight, float factor) {
        LimelightHelpers.SetFiducialDownscalingOverride(limelight.name(), factor);
    }

    /** Applies the same downscale factor to every camera. */
    public void setDownscalingAll(float factor) {
        for (Limelight ll : config.limelights)
            LimelightHelpers.SetFiducialDownscalingOverride(ll.name(), factor);
    }

    // ── Rewind ────────────────────────────────────────────────────────────────

    /** Enables or pauses the rewind buffer recording on one camera. */
    public void setRewindEnabled(Limelight limelight, boolean enabled) {
        LimelightHelpers.setRewindEnabled(limelight.name(), enabled);
    }

    /**
     * Triggers a rewind capture on one camera saving up to {@code durationSeconds}
     * of buffered footage (maximum 165 s).
     */
    public void triggerRewind(Limelight limelight, double durationSeconds) {
        LimelightHelpers.triggerRewindCapture(limelight.name(), durationSeconds);
    }

    // ── Port forwarding ───────────────────────────────────────────────────────

    /**
     * Enables port forwarding for a network-connected Limelight so its web interface
     * and video stream are reachable from the driver station. Uses the IP stored in
     * {@link Limelight#ip()}. Call once during robot initialization.
     */
    public void enablePortForwarding(Limelight limelight) {
        String ip = limelight.ip();
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, ip, port);
        }
    }

    /** Calls {@link #enablePortForwarding(Limelight)} for every camera in the config. */
    public void enablePortForwardingAll() {
        for (Limelight ll : config.limelights) enablePortForwarding(ll);
    }

    /**
     * Enables port forwarding for a USB-connected Limelight 3A/3G.
     * {@code usbIndex} 0 → ports 5800–5809 forward to {@code 172.29.0.1},
     * index 1 → ports 5810–5819, etc. Call once during robot initialization.
     */
    public static void enablePortForwardingUSB(int usbIndex) {
        LimelightHelpers.setupPortForwardingUSB(usbIndex);
    }

    // ── Crop window ───────────────────────────────────────────────────────────

    /**
     * Crops the image on one camera to the bounding box of the AprilTags currently
     * visible in the latest input frame, with a small margin. Reduces the pixel area
     * the detector must scan, improving CPU performance when tags stay in frame.
     * Resets to full frame if no tags are currently visible.
     */
    public void autoCropToTags(Limelight limelight) {
        Integer idx = limelightIndex.get(limelight.name());
        if (idx == null) return;

        double[] rawFids = inputs[idx].rawFiducials;
        int tagCount = rawFids.length / 7;

        if (tagCount == 0) {
            LimelightHelpers.setCropWindow(limelight.name(), -1, 1, -1, 1);
            return;
        }

        double hfovHalf = config.simulatedFOVDegrees / 2.0;
        double vfovHalf = config.verticalFOVDegrees / 2.0;

        double minX = Double.MAX_VALUE,  maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE,  maxY = -Double.MAX_VALUE;

        for (int i = 0; i < tagCount; i++) {
            int base = i * 7;
            double normX = rawFids[base + 1] / hfovHalf; // txnc → [-1,1]
            double normY = rawFids[base + 2] / vfovHalf; // tync → [-1,1]
            if (normX < minX) minX = normX;
            if (normX > maxX) maxX = normX;
            if (normY < minY) minY = normY;
            if (normY > maxY) maxY = normY;
        }

        double pad = 0.3;
        LimelightHelpers.setCropWindow(limelight.name(),
                Math.max(-1, minX - pad), Math.min(1, maxX + pad),
                Math.max(-1, minY - pad), Math.min(1, maxY + pad));
    }

    /** Calls {@link #autoCropToTags(Limelight)} for every camera in the config. */
    public void autoCropToTagsAll() {
        for (Limelight ll : config.limelights) autoCropToTags(ll);
    }

    /** Resets the crop window on one camera to the full frame. */
    public void resetCrop(Limelight limelight) {
        LimelightHelpers.setCropWindow(limelight.name(), -1, 1, -1, 1);
    }

    /** Resets the crop window to the full frame on every camera. */
    public void resetCropAll() {
        for (Limelight ll : config.limelights) LimelightHelpers.setCropWindow(ll.name(), -1, 1, -1, 1);
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /**
     * Reconstructs a {@link LimelightHelpers.PoseEstimate} from the already-logged inputs,
     * without touching NetworkTables again.
     */
    private LimelightHelpers.PoseEstimate parsePoseEstimate(
            LimelightIOInputsAutoLogged inp, PoseEstimationStrategy strategy) {
        boolean isMT2 = strategy == PoseEstimationStrategy.MEGATAG_2;
        double[] arr = isMT2 ? inp.botpose_orb_wpiblue : inp.botpose_wpiblue;
        double ntTimestamp = isMT2 ? inp.orbTimestampSeconds : inp.timestampSeconds;

        if (arr.length < 11) return new LimelightHelpers.PoseEstimate();

        Pose2d pose = new Pose2d(new Translation2d(arr[0], arr[1]),
                Rotation2d.fromDegrees(arr[5]));
        double latency  = arr[6];
        int    tagCount = (int) arr[7];
        double tagSpan  = arr[8];
        double avgDist  = arr[9];
        double avgArea  = arr[10];

        // Timestamp: NT server time (seconds) minus total capture+pipeline latency.
        double adjustedTimestamp = ntTimestamp - (latency / 1000.0);

        LimelightHelpers.RawFiducial[] rawFiducials;
        int expectedTotal = 11 + 7 * tagCount;
        if (tagCount == 0 || arr.length < expectedTotal) {
            rawFiducials = new LimelightHelpers.RawFiducial[0];
        } else {
            rawFiducials = new LimelightHelpers.RawFiducial[tagCount];
            for (int i = 0; i < tagCount; i++) {
                int b = 11 + i * 7;
                rawFiducials[i] = new LimelightHelpers.RawFiducial(
                        (int) arr[b], arr[b+1], arr[b+2], arr[b+3],
                        arr[b+4], arr[b+5], arr[b+6]);
            }
        }

        return new LimelightHelpers.PoseEstimate(
                pose, adjustedTimestamp, latency, tagCount,
                tagSpan, avgDist, avgArea, rawFiducials, isMT2);
    }

    private VisionType classify(
            LimelightHelpers.PoseEstimate estimate,
            double ambiguity,
            double yawRateDps,
            double linearVelocityMps) {
        if (estimate == null || estimate.tagCount == 0 || estimate.pose == null) {
            return VisionType.REJECTED_INVALID;
        }

        if (config.fieldArea != null && !config.fieldArea.contains(estimate.pose)) {
            return VisionType.REJECTED_INVALID;
        }

        if (estimate.tagCount == 1
                && !config.useDisableAmbiguityRejection
                && ambiguity > config.maxAmbiguity) {
            return VisionType.REJECTED_AMBIGUITY;
        }

        if (Math.abs(yawRateDps) > config.maxAngularVelocityDegPerSec
                || linearVelocityMps > config.maxLinearVelocityMetersPerSec) {
            return VisionType.REJECTED_MOVEMENT;
        }

        return VisionType.ACCEPTED;
    }
}
