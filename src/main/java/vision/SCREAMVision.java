package vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import util.GeomUtil;
import vision.LimelightHelpers.PoseEstimate;
import vision.LimelightVision.Limelight;
import zones.RectangularPoseArea;

public class SCREAMVision extends SubsystemBase{
    
    public static class SCREAMVisionConfiguration{

        public boolean enabled = true;
        public boolean logTelemetry = true;

        public boolean simEnabled = true;
        public boolean forceSimulation = false;

        public boolean simDrawWireframe = false;
        public int resWidth = 1280;
        public int resHeight = 960;
        public Rotation2d fovDiag = Rotation2d.fromDegrees(91.145);
        public int fps = 15;

        public Limelight[] limelights = null;

        public AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        public RectangularPoseArea fieldArea = new RectangularPoseArea(Translation2d.kZero, new Translation2d(fieldLayout.getFieldLength(), fieldLayout.getFieldWidth()));

        public double xyStdBaseline = 0.93;
        public double thetaStdBaseline = 12.5;
    }

    protected final SCREAMVisionConfiguration config;
    protected final Limelight[] limelights;

    private VisionSystemSim visionSim;
    private List<PhotonCamera> simCameras;

    private final Map<String, Double> lastValidTimestamps = new HashMap<>();

    private final Supplier<Pose2d> robotPose;

    public SCREAMVision(SCREAMVisionConfiguration config, Supplier<Pose2d> robotPose){
        this.config = config;
        this.limelights = config.limelights;

        this.robotPose = robotPose;

        if(shouldSimulate()){
            initializeSimulation();
        }
    }

    public boolean shouldSimulate(){
        return (config.simEnabled && RobotBase.isSimulation()) || config.forceSimulation;
    }

    public void initializeSimulation(){
        visionSim = new VisionSystemSim("main");

        visionSim.addAprilTags(config.fieldLayout);

        var cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(
            config.resWidth,
            config.resHeight,
            config.fovDiag);
        cameraProps.setCalibError(0.35, 0.10);
        cameraProps.setFPS(config.fps);
        cameraProps.setAvgLatencyMs(10);
        cameraProps.setLatencyStdDevMs(3);

        for(Limelight ll : limelights){
            var camera = new PhotonCamera(ll.name());

            simCameras.add(camera);

            var simCamera = new PhotonCameraSim(camera, cameraProps);
            simCamera.enableProcessedStream(true);
            simCamera.enableRawStream(true);
            simCamera.enableDrawWireframe(config.simDrawWireframe);

            visionSim.addCamera(simCamera, GeomUtil.pose3dToTransform3d(ll.relativePosition()));
        }
    }

    public Optional<PoseEstimate> getMt1Estimate(Limelight limelight){
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name());
        if (!rejectEstimate(estimate, limelight)) {
            if (config.logTelemetry) {
                lastValidTimestamps.put(limelight.name(), estimate.timestampSeconds);
                double age = Timer.getFPGATimestamp() - estimate.timestampSeconds;
                DogLog.log("Vision/" + limelight.name() + "/PoseEstimate", estimate.pose);
                DogLog.log("Vision/" + limelight.name() + "/EstimateType", "Megatag 1");
                DogLog.log("Vision/" + limelight.name() + "/Age", age);
            }
            return Optional.of(estimate);
        } else {
            if (config.logTelemetry) {
                Double lastValidTimestamp = lastValidTimestamps.get(limelight.name());
                if (lastValidTimestamp != null) {
                    double age = Timer.getFPGATimestamp() - lastValidTimestamp;
                    DogLog.log("Vision/" + limelight.name() + "/Age", age);
                } else {
                    DogLog.log("Vision/" + limelight.name() + "/Age", Double.NaN);
                }
            }
            return Optional.empty();
        }
    }

    public List<PoseEstimate> getValidMt1Estimates(){
        List<PoseEstimate> validEstimates = new ArrayList<>();
        for(int i = 0; i < limelights.length; i++){
            Optional<PoseEstimate> estimate = getMt1Estimate(limelights[i]);
            if(estimate.isPresent()){
                validEstimates.add(estimate.get());
            }
        }
        return validEstimates;
    }

    public Optional<PoseEstimate> getMt2Estimate(Limelight limelight, Rotation2d robotHeading, Rotation2d yawRate){
        LimelightHelpers.SetRobotOrientation(limelight.name(), robotHeading.getDegrees(), yawRate.getDegrees(), 0, 0, 0, 0);
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name());
        if (!rejectEstimate(estimate, limelight)) {
            if (config.logTelemetry) {
                lastValidTimestamps.put(limelight.name(), estimate.timestampSeconds);
                double age = Timer.getFPGATimestamp() - estimate.timestampSeconds;
                DogLog.log("Vision/" + limelight.name() + "/PoseEstimate", estimate.pose);
                DogLog.log("Vision/" + limelight.name() + "/EstimateType", "Megatag 2");
                DogLog.log("Vision/" + limelight.name() + "/Age", age);
            }
            return Optional.of(estimate);
        } else {
            if (config.logTelemetry) {
                Double lastValidTimestamp = lastValidTimestamps.get(limelight.name());
                if (lastValidTimestamp != null) {
                    double age = Timer.getFPGATimestamp() - lastValidTimestamp;
                    DogLog.log("Vision/" + limelight.name() + "/Age", age);
                } else {
                    DogLog.log("Vision/" + limelight.name() + "/Age", Double.NaN);
                }
            }
            return Optional.empty();
        }
    }

    public List<PoseEstimate> getValidMt2Estimates(Rotation2d robotHeading, Rotation2d yawRate){
        List<PoseEstimate> validEstimates = new ArrayList<>();
        for(int i = 0; i < limelights.length; i++){
            Optional<PoseEstimate> estimate = getMt2Estimate(limelights[i], robotHeading, yawRate);
            if(estimate.isPresent()){
                validEstimates.add(estimate.get());
            }
        }
        return validEstimates;
    }

  private boolean rejectEstimate(PoseEstimate estimate, Limelight limelight) {
    if (estimate == null
        || estimate.tagCount == 0
        || !config.fieldArea.contains(estimate.pose)) {
      DogLog.log("Vision/" + limelight.name() + "/RejectedType", "Invalid");
      return true;
    } else if ((estimate.tagCount == 1 && estimate.rawFiducials[0].ambiguity > 0.3)) {
      DogLog.log("Vision/" + limelight.name() + "/RejectedType", "Ambiguity");
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
      if(((config.simEnabled && RobotBase.isSimulation()) || config.forceSimulation) && visionSim != null){
        visionSim.update(robotPose.get());
      for (int i = 0; i < limelights.length; i++) {
        for (PhotonPipelineResult result : simCameras.get(i).getAllUnreadResults()) {
          writeToTable(
              result,
              NetworkTableInstance.getDefault().getTable(limelights[i].name()),
              GeomUtil.pose3dToTransform3d(limelights[i].relativePosition()).inverse());
        }
      }
      }
  }

  private void writeToTable(
      PhotonPipelineResult result, NetworkTable table, Transform3d cameraToRobot) {
    double latencyMs = (Timer.getFPGATimestamp() - result.getTimestampSeconds()) * 1000.0;
    if (result.getMultiTagResult().isPresent()) {
      MultiTargetPNPResult multiTagResult = result.getMultiTagResult().get();
      Transform3d best = multiTagResult.estimatedPose.best.plus(cameraToRobot);
      Pose2d fieldToCamera =
          new Pose2d(best.getTranslation().toTranslation2d(), best.getRotation().toRotation2d());

      int targetCount = result.targets.size();
      List<Double> pose_data = new ArrayList<>(11);
      List<Double> rawFiducial_data = new ArrayList<>(targetCount * 7);

      pose_data.addAll(
          Arrays.asList(
              best.getX(),
              best.getY(),
              best.getZ(),
              0.0, // roll
              0.0, // pitch
              fieldToCamera.getRotation().getDegrees(),
              latencyMs,
              (double) multiTagResult.fiducialIDsUsed.size(),
              0.0, // tag span
              calculateAverageTagDistance(result), // avg tag dist
              result.getBestTarget().getArea()));

      for (PhotonTrackedTarget target : result.targets) {
        rawFiducial_data.add((double) target.getFiducialId());
        rawFiducial_data.add(target.getYaw());
        rawFiducial_data.add(target.getPitch());
        rawFiducial_data.add(target.getArea()); // ta
        rawFiducial_data.add(
            target.getBestCameraToTarget().getTranslation().getNorm()); // distToCamera
        rawFiducial_data.add(
            target
                .getBestCameraToTarget()
                .plus(cameraToRobot)
                .getTranslation()
                .getNorm()); // distToRobot
        rawFiducial_data.add(target.getPoseAmbiguity()); // ambiguity
      }

      double[] poseArray = pose_data.stream().mapToDouble(Double::doubleValue).toArray();
      double[] rawFiducialArray =
          rawFiducial_data.stream().mapToDouble(Double::doubleValue).toArray();
      table.getEntry("rawfiducials").setDoubleArray(rawFiducialArray);
      table.getEntry("botpose_wpiblue").setDoubleArray(poseArray);
      table.getEntry("botpose_orb_wpiblue").setDoubleArray(poseArray);
    }

    table.getEntry("tv").setInteger(result.hasTargets() ? 1 : 0);
    table
        .getEntry("cl")
        .setDouble((Timer.getFPGATimestamp() - result.getTimestampSeconds()) * 1000.0);
  }

  private double calculateAverageTagDistance(PhotonPipelineResult result) {
    double distance = 0;
    for (PhotonTrackedTarget target : result.targets) {
      distance += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    return distance / result.targets.size();
  }

}
