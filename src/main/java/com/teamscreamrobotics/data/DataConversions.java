package com.teamscreamrobotics.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Utility methods for converting geometry and kinematics objects to primitive arrays and back. */
public class DataConversions {

  /**
   * Converts a {@link Pose2d} to a {@code [x, y, headingDegrees]} array.
   *
   * @param pose the pose to convert
   */
  public static double[] pose2dToArray(Pose2d pose) {
    return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
  }

  /**
   * Converts a {@link Translation3d} to a 7-element array {@code [x, y, z, 0, 0, 0, 0]}
   * (position only, quaternion zeroed).
   *
   * @param translation the translation to convert
   */
  public static double[] translation3dToArray(Translation3d translation) {
    return new double[] {translation.getX(), translation.getY(), translation.getZ(), 0, 0, 0, 0};
  }

  /**
   * Converts a {@link Translation3d} and {@link Rotation3d} to a 7-element pose array
   * {@code [x, y, z, qx, qy, qz, qw]}.
   *
   * @param translation the position
   * @param rot         the rotation (converted to quaternion)
   */
  public static double[] translation3dToArray(Translation3d translation, Rotation3d rot) {
    Quaternion quat = rot.getQuaternion();
    return new double[] {
      translation.getX(),
      translation.getY(),
      translation.getZ(),
      quat.getX(),
      quat.getY(),
      quat.getZ(),
      quat.getW()
    };
  }

  /**
   * Extracts the first three elements of a pose array as a {@link Translation3d}.
   *
   * @param array array with at least 3 elements: {@code [x, y, z, ...]}
   */
  public static Translation3d pose3dArrayToTranslation3d(double[] array) {
    return new Translation3d(array[0], array[1], array[2]);
  }

  /**
   * Packs an array of {@link Translation3d} objects into a flat {@code double[]} with 7 slots per
   * translation (x, y, z at indices 0–2; remaining 4 slots zeroed).
   *
   * @param translations the translations to pack
   */
  public static double[] translation3dArrayToNumArray(Translation3d[] translations) {
    double[] combinedArr = new double[translations.length * 7];
    int index = 0;

    for (Translation3d translation : translations) {
      combinedArr[index] = translation.getX();
      combinedArr[index + 1] = translation.getY();
      combinedArr[index + 2] = translation.getZ();

      index += 7;
      /* double[] arr = translation3dToArray(translation);

      System.arraycopy(arr, 0, combinedArr, index, arr.length);

      index += arr.length; */
    }

    return combinedArr;
  }

  /**
   * Packs an array of {@link Translation2d} objects into a flat {@code double[]} with 7 slots per
   * translation (x, y at indices 0–1; remaining 5 slots zeroed).
   *
   * @param translations the translations to pack
   */
  public static double[] translation2dArrayToNumArray(Translation2d[] translations) {
    double[] combinedArr = new double[translations.length * 7];
    int index = 0;

    for (Translation2d translation : translations) {
      combinedArr[index] = translation.getX();
      combinedArr[index + 1] = translation.getY();

      index += 7;
    }

    return combinedArr;
  }

  /**
   * Packs an array of {@link Translation2d} objects into a flat {@code double[]} with 7 slots per
   * translation, inserting a constant {@code z} value at index 2.
   *
   * @param translations the translations to pack
   * @param z            z value to insert at index 2 of each slot
   */
  public static double[] translation2dArrayToNumArray(Translation2d[] translations, double z) {
    double[] combinedArr = new double[translations.length * 7];
    int index = 0;

    for (Translation2d translation : translations) {
      combinedArr[index] = translation.getX();
      combinedArr[index + 1] = translation.getY();
      combinedArr[index + 2] = z;

      index += 7;
    }

    return combinedArr;
  }

  /**
   * Converts a {@link ChassisSpeeds} to a {@code [vx, vy, omega]} array.
   *
   * @param speeds the chassis speeds to convert
   */
  public static double[] chassisSpeedsToArray(ChassisSpeeds speeds) {
    return new double[] {
      speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond
    };
  }

  /**
   * Projects a {@link Translation3d} onto the XZ plane, returning {@code (x, z)} as a {@link Translation2d}.
   *
   * @param translation the 3D translation to project
   */
  public static Translation2d projectTo2d(Translation3d translation) {
    return new Translation2d(translation.getX(), translation.getZ());
  }

  /**
   * Lifts a {@link Translation2d} into 3D as {@code (x, 0, y)}.
   *
   * @param translation the 2D translation to lift
   */
  public static Translation3d projectTo3d(Translation2d translation){
    return new Translation3d(translation.getX(), 0, translation.getY());
  }
}
