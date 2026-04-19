package com.teamscreamrobotics.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Various utility methods */
public class ScreamUtil {
  /** Default epsilon used by the no-argument {@link #epsilonEquals} overloads. */
  public static final double EPSILON = 1e-3;

  /** Wraps {@code rotation} to the range {@code (-π, π]}. */
  public static Rotation2d boundRotation(Rotation2d rotation) {
    return new Rotation2d(MathUtil.angleModulus(rotation.getRadians()));
  }

  /** Wraps {@code rotation} to the range {@code [0°, 360°)}. */
  public static Rotation2d boundRotation0_360(Rotation2d rotation) {
    rotation = boundRotation(rotation);
    if (rotation.getDegrees() < 0) return Rotation2d.fromDegrees(rotation.getDegrees() + 360.0);
    return rotation;
  }

  /**
   * Returns the tangent (heading) angle of the vector from {@code start} to {@code end}.
   *
   * @param start the starting point
   * @param end   the ending point
   */
  public static Rotation2d getTangent(Translation2d start, Translation2d end) {
    Translation2d dist = end.minus(start);
    return new Rotation2d(dist.getX(), dist.getY());
  }

  /**
   * Returns {@code true} if {@code |a - b| <= epsilon}.
   *
   * @param a       first value
   * @param b       second value
   * @param epsilon tolerance
   */
  public static boolean epsilonEquals(double a, double b, final double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  /** Returns {@code true} if {@code |a - b| <= EPSILON}. */
  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, EPSILON);
  }

  /** Returns {@code true} if {@code |a - b| <= epsilon} (integer overload). */
  public static boolean epsilonEquals(int a, int b, int epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  /**
   * Estimates output torque at the given speed using a linear motor model.
   *
   * @param stallTorque maximum torque at zero speed (N·m)
   * @param freeSpeed   no-load speed (rad/s or RPM — must match units of {@code speed})
   * @param speed       current speed
   */
  public double getStallTorque(double stallTorque, double freeSpeed, double speed) {
    return -stallTorque / freeSpeed * speed + stallTorque;
  }

  /**
   * Converts a {@link ChassisSpeeds} to a {@link Twist2d} with the same vx, vy, and omega.
   *
   * @param chassisSpeeds the speeds to convert
   */
  public static Twist2d chassisSpeedsToTwist2d(ChassisSpeeds chassisSpeeds) {
    return new Twist2d(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Returns {@code true} if all components of two twists are within {@code epsilon} of each other.
   *
   * @param twist   first twist
   * @param other   second twist
   * @param epsilon per-component tolerance
   */
  public static boolean epsilonEquals(final Twist2d twist, final Twist2d other, double epsilon) {
    return ScreamUtil.epsilonEquals(twist.dx, other.dx, epsilon)
        && ScreamUtil.epsilonEquals(twist.dy, other.dy, epsilon)
        && ScreamUtil.epsilonEquals(twist.dtheta, other.dtheta, epsilon);
  }

  /** Returns {@code true} if all twist components are within {@link #EPSILON} of each other. */
  public static boolean epsilonEquals(final Twist2d twist, final Twist2d other) {
    return epsilonEquals(twist, other, EPSILON);
  }

  /**
   * Returns {@code true} if {@code lower < value < upper} (exclusive).
   *
   * @param value the value to test
   * @param upper the exclusive upper bound
   * @param lower the exclusive lower bound
   */
  public static boolean valueBetween(double value, double upper, double lower) {
    return value < upper && value > lower;
  }

  /**
   * Returns {@code true} if {@code currentAngle} is within {@code threshold} of {@code targetAngle},
   * accounting for wrap-around.
   *
   * @param targetAngle  the desired angle
   * @param currentAngle the measured angle
   * @param threshold    the maximum allowable error
   */
  public static boolean withinAngleThreshold(
      Rotation2d targetAngle, Rotation2d currentAngle, Rotation2d threshold) {
    return MathUtil.isNear(
        targetAngle.getDegrees(), currentAngle.getDegrees(), threshold.getDegrees(), -180, 180);
  }

  /**
   * Returns a random double in {@code [lower, upper]}, swapping bounds if {@code lower >= upper}.
   *
   * @param lower lower bound
   * @param upper upper bound
   */
  public static double random(double lower, double upper) {
    if (lower >= upper) {
      double temp = upper;
      upper = lower;
      lower = temp;
    }
    return (Math.random() * (upper - lower + 1)) + lower;
  }
}
