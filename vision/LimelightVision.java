package com.SCREAMLib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

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

  public static Pose2d getBotPose2d(Limelight limelight) {
    return LimelightHelpers.getBotPose2d_wpiBlue(limelight.name);
  }

  public static Pose3d getBotPose3d(Limelight limelight) {
    return LimelightHelpers.getBotPose3d_wpiBlue(limelight.name);
  }

  public static Pose2d getBotPose2d_TargetSpace(Limelight limelight) {
    return LimelightHelpers.getBotPose3d_TargetSpace(limelight.name).toPose2d();
  }

  public static Pose3d getBotPose3d_TargetSpace(Limelight limelight) {
    return LimelightHelpers.getBotPose3d_TargetSpace(limelight.name);
  }

  public static double getDistanceToTargetMeters(double targetHeight, Limelight limelight) {
    double goal_theta =
        limelight.relativePosition.getRotation().getY() + Math.toRadians(getTY(limelight));
    double height_diff = targetHeight - limelight.relativePosition.getZ();

    return height_diff / Math.tan(goal_theta);
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
}
