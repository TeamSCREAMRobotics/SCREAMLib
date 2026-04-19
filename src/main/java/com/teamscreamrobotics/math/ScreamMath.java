package com.teamscreamrobotics.math;

import com.teamscreamrobotics.data.Length;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

/** General-purpose math utilities for FRC calculations. */
public class ScreamMath {

  /** Conversion factor: meters per inch ({@value} m/in). */
  public static final double METERS_PER_INCH = 0.0254;

  /**
   * Returns the arithmetic mean of the given values, or {@code 0.0} if the array is empty.
   *
   * @param nums the values to average
   */
  public static double average(double... nums) {
    if (nums.length == 0) return 0.0;

    double sum = 0.0;
    for (double num : nums) {
      sum += num;
    }
    return sum / nums.length;
  }

  /**
   * Linearly maps {@code value} from one range to another.
   *
   * @param value    the input value in the source range
   * @param fromLow  lower bound of the source range
   * @param fromHigh upper bound of the source range
   * @param toLow    lower bound of the target range
   * @param toHigh   upper bound of the target range
   * @throws IllegalArgumentException if the source range has zero width
   */
  public static double mapRange(
      double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    if (fromHigh - fromLow == 0) {
      throw new IllegalArgumentException("Input range has zero width");
    }

    return toLow + ((value - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow);
  }

  /**
   * Returns the translational speed magnitude (vx² + vy²)^½ from a {@link ChassisSpeeds}.
   *
   * @param speeds the chassis speed
   */
  public static double getLinearVelocity(ChassisSpeeds speeds) {
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  /**
   * Rotates a 3D point around the Z-axis by {@code yaw}, preserving Z.
   *
   * @param point the point to rotate
   * @param yaw   the rotation angle around the Z-axis
   */
  public static Translation3d rotatePoint(Translation3d point, Rotation2d yaw) {
    double cosAngle = yaw.getCos();
    double sinAngle = yaw.getSin();

    double newX = point.getX() * cosAngle - point.getY() * sinAngle;
    double newY = point.getX() * sinAngle + point.getY() * cosAngle;
    double newZ = point.getZ();

    return new Translation3d(newX, newY, newZ);
  }

  /**
   * Rotates a 2D point by {@code yaw}.
   *
   * @param point the point to rotate
   * @param yaw   the rotation angle
   */
  public static Translation2d rotatePoint(Translation2d point, Rotation2d yaw) {
    Translation3d t3d = rotatePoint(new Translation3d(point.getX(), point.getY(), 0), yaw);
    return t3d.toTranslation2d();
  }

  /**
   * Returns the bearing angle from {@code current} to {@code target}.
   *
   * @param current the observer position
   * @param target  the target position
   */
  public static Rotation2d calculateAngleToPoint(Translation2d current, Translation2d target) {
    double targetX = target.getX() - current.getX();
    double targetY = target.getY() - current.getY();
    return Rotation2d.fromRadians(Math.atan2(targetY, targetX));
  }

  /**
   * Clamps {@code rotation} to the range {@code [low, high]} in radians.
   *
   * @param rotation the rotation to clamp
   * @param high     the upper bound
   * @param low      the lower bound
   */
  public static Rotation2d clamp(Rotation2d rotation, Rotation2d high, Rotation2d low) {
    return Rotation2d.fromRadians(
        MathUtil.clamp(rotation.getRadians(), low.getRadians(), high.getRadians()));
  }

  /** Returns {@code n²}. */
  public static double square(double n){
    return Math.pow(n, 2);
  }

  /**
   * Applies the parallel axis theorem: {@code I = I_cm + m * d²}.
   *
   * @param moi      moment of inertia about the center of mass
   * @param mass     mass of the object
   * @param distance distance from the center of mass to the new axis
   * @return the moment of inertia about the new parallel axis
   */
  public static MomentOfInertia parallelAxisTheorem(MomentOfInertia moi, Mass mass, Length distance){
    return Units.KilogramSquareMeters.of(moi.in(Units.KilogramSquareMeters) * mass.times(distance.squared().getMeters()).in(Units.Kilograms));
  }
}
