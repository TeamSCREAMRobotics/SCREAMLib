package com.SCREAMLib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ScreamMath {

  public static double average(double... nums) {
    if (nums.length == 0) return 0.0;

    double sum = 0.0;
    for (double num : nums) {
      sum += num;
    }
    return sum / nums.length;
  }

  public static double mapRange(
      double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    if (fromHigh - fromLow == 0) {
      throw new IllegalArgumentException("Input range has zero width");
    }

    return toLow + ((value - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow);
  }

  public static double getLinearVelocity(ChassisSpeeds speeds) {
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public static Translation3d rotatePoint(Translation3d point, Rotation2d yaw) {
    double cosAngle = yaw.getCos();
    double sinAngle = yaw.getSin();

    double newX = point.getX() * cosAngle - point.getY() * sinAngle;
    double newY = point.getX() * sinAngle + point.getY() * cosAngle;
    double newZ = point.getZ();

    return new Translation3d(newX, newY, newZ);
  }

  public static Rotation2d calculateAngleToPoint(Translation2d current, Translation2d target) {
    double targetX = target.getX() - current.getX();
    double targetY = target.getY() - current.getY();
    return Rotation2d.fromRadians(Math.atan2(targetY, targetX));
  }
}
