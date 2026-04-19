package com.teamscreamrobotics.vision;

import com.teamscreamrobotics.data.Length;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.teamscreamrobotics.vision.LimelightHelpers.PoseEstimate;

/** Typed wrapper around {@link LimelightHelpers} providing vision measurements and camera control. */
public class LimelightVision {

  /** Identifies a Limelight camera by NetworkTables name and its pose relative to the robot. */
  public record Limelight(String name, Pose3d relativePosition) {}

  /** LED control modes supported by Limelight cameras. */
  public enum LEDMode {
    OFF,
    ON,
    BLINK;
  }

  /** Returns the horizontal offset angle (TX) to the primary target in degrees. */
  public static double getTX(Limelight limelight) {
    return LimelightHelpers.getTX(limelight.name);
  }

  /** Returns the vertical offset angle (TY) to the primary target in degrees. */
  public static double getTY(Limelight limelight) {
    return LimelightHelpers.getTY(limelight.name);
  }

  /** Returns the target area (TA) as a percentage of the image (0–100). */
  public static double getTA(Limelight limelight) {
    return LimelightHelpers.getTA(limelight.name);
  }

  /** Returns {@code true} if the camera has a valid target. */
  public static boolean getTV(Limelight limelight) {
    return LimelightHelpers.getTV(limelight.name);
  }

  /** Returns pipeline processing latency in milliseconds. */
  public static double getLatency_Pipeline(Limelight limelight) {
    return LimelightHelpers.getLatency_Pipeline(limelight.name);
  }

  /** Returns image capture latency in milliseconds. */
  public static double getLatency_Capture(Limelight limelight) {
    return LimelightHelpers.getLatency_Capture(limelight.name);
  }

  /** Returns total latency (pipeline + capture) in milliseconds. */
  public static double getLatency(Limelight limelight){
    return getLatency_Pipeline(limelight) + getLatency_Capture(limelight);
  }

  /**
   * Sets robot orientation and returns a MegaTag2 pose estimate in WPILib blue-origin coordinates.
   *
   * @param robotHeadingDegrees current robot yaw in degrees
   * @param yawRateDps          current yaw rate in degrees per second
   */
  public static PoseEstimate getPoseEstimate_MT2(
      Limelight limelight, double robotHeadingDegrees, double yawRateDps) {
    LimelightHelpers.SetRobotOrientation(
        limelight.name, robotHeadingDegrees, yawRateDps, 0, 0, 0, 0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name);
  }

  /**
   * Estimates horizontal ground distance to a target using the TY angle and known heights.
   *
   * @param targetHeight the known height of the target
   * @param limelight    the camera to use
   */
  public static Length getDistanceToTargetTYBased(Length targetHeight, Limelight limelight) {
    double goal_theta =
        limelight.relativePosition.getRotation().getY() + Math.toRadians(getTY(limelight));
    double height_diff = targetHeight.getMeters() - limelight.relativePosition.getZ();

    return Length.fromMeters(height_diff / Math.tan(goal_theta));
  }

  /** Returns the 3D Euclidean distance to the primary target in camera space. */
  public static Length get3D_DistanceToTarget(Limelight limelight){
    return Length.fromMeters(LimelightHelpers.getTargetPose3d_CameraSpace(limelight.name).getTranslation().getNorm());
  }

  /** Returns the robot-relative horizontal angle to the target, accounting for camera offset. */
  public static Rotation2d getAngleToTargetTXBased(Limelight limelight) {
    return Rotation2d.fromDegrees(
        -getTX(limelight) - Math.toDegrees(limelight.relativePosition.getRotation().getZ()));
  }

  /** Returns the currently active pipeline index. */
  public static int getCurrentPipeline(Limelight limelight) {
    return (int) LimelightHelpers.getCurrentPipelineIndex(limelight.name);
  }

  /** Sets the camera LED to the given {@link LEDMode}. */
  public static void setLEDMode(LEDMode ledMode, Limelight limelight) {
    switch (ledMode) {
      case OFF:
        LimelightHelpers.setLEDMode_ForceOff(limelight.name);
        break;
      case ON:
        LimelightHelpers.setLEDMode_ForceOn(limelight.name);
        break;
      case BLINK:
        LimelightHelpers.setLEDMode_ForceBlink(limelight.name);
        break;
    }
  }

  /** Sets the AprilTag ID that the camera should prioritize when multiple tags are visible. */
  public static void setPriorityTagID(int id, Limelight limelight) {
    LimelightHelpers.setPriorityTagID(limelight.name, id);
  }

  /** Switches the camera to the pipeline at the given zero-based index. */
  public static void setPipeline(int index, Limelight limelight) {
    LimelightHelpers.setPipelineIndex(limelight.name, index);
  }

  /**
   * Restricts the active image region to reduce processing load.
   *
   * @param cropXMin left edge (−1 to 1)
   * @param cropXMax right edge (−1 to 1)
   * @param cropYMin bottom edge (−1 to 1)
   * @param cropYMax top edge (−1 to 1)
   */
  public static void setCropWindow(
      double cropXMin, double cropXMax, double cropYMin, double cropYMax, Limelight limelight) {
    LimelightHelpers.setCropWindow(limelight.name, cropXMin, cropXMax, cropYMin, cropYMax);
  }

  /**
   * Sets the camera frame throttle — higher values skip more frames to reduce CPU load.
   *
   * @param skipFrames number of frames to skip between processed frames (0 = no throttle)
   */
  public static void setThrottle(int skipFrames, Limelight limelight){
    NetworkTable llt = NetworkTableInstance.getDefault().getTable(limelight.name);
    llt.getEntry("throttle_set").setNumber(skipFrames);
  }
}
