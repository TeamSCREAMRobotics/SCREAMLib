package com.SCREAMLib.vision;

import com.SCREAMLib.data.Length;
import com.SCREAMLib.vision.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightVision {

  public record Limelight(String name, Pose3d relativePosition) {}

  public enum LEDMode {
    OFF,
    ON,
    BLINK;
  }

  public static double getTX(Limelight limelight) {
    return LimelightHelpers.getTX(limelight.name);
  }

  public static double getTY(Limelight limelight) {
    return LimelightHelpers.getTY(limelight.name);
  }

  public static double getTA(Limelight limelight) {
    return LimelightHelpers.getTA(limelight.name);
  }

  public static boolean getTV(Limelight limelight) {
    return LimelightHelpers.getTV(limelight.name);
  }

  public static double getLatency_Pipeline(Limelight limelight) {
    return LimelightHelpers.getLatency_Pipeline(limelight.name);
  }

  public static double getLatency_Capture(Limelight limelight) {
    return LimelightHelpers.getLatency_Capture(limelight.name);
  }

  public static PoseEstimate getPoseEstimate_MT2(
      Limelight limelight, double robotHeadingDegrees, double yawRateDps) {
    LimelightHelpers.SetRobotOrientation(
        limelight.name, robotHeadingDegrees, yawRateDps, 0, 0, 0, 0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name);
  }

  public static Length getDistanceToTargetTYBased(Length targetHeight, Limelight limelight) {
    double goal_theta =
        limelight.relativePosition.getRotation().getY() + Math.toRadians(getTY(limelight));
    double height_diff = targetHeight.getMeters() - limelight.relativePosition.getZ();

    return Length.fromMeters(height_diff / Math.tan(goal_theta));
  }

  public static Rotation2d getAngleToTargetTXBased(Limelight limelight) {
    return Rotation2d.fromDegrees(
        -getTX(limelight) - Math.toDegrees(limelight.relativePosition.getRotation().getZ()));
  }

  public static int getCurrentPipeline(Limelight limelight) {
    return (int) LimelightHelpers.getCurrentPipelineIndex(limelight.name);
  }

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

  public static void setPriorityTagID(int id, Limelight limelight) {
    LimelightHelpers.setPriorityTagID(limelight.name, id);
  }

  public static void setPipeline(int index, Limelight limelight) {
    LimelightHelpers.setPipelineIndex(limelight.name, index);
  }

  public static void setCropWindow(
      double cropXMin, double cropXMax, double cropYMin, double cropYMax, Limelight limelight) {
    LimelightHelpers.setCropWindow(limelight.name, cropXMin, cropXMax, cropYMin, cropYMax);
  }
}
