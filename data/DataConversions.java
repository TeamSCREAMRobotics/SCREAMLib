package com.SCREAMLib.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DataConversions {

  public static double[] pose2dToArray(Pose2d pose) {
    return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
  }

  public static double[] translation3dToArray(Translation3d translation) {
    return new double[] {translation.getX(), translation.getY(), translation.getZ(), 0, 0, 0, 0};
  }

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

  public static Translation3d pose3dArrayToTranslation3d(double[] array) {
    return new Translation3d(array[0], array[1], array[2]);
  }

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

  public static double[] chassisSpeedsToArray(ChassisSpeeds speeds) {
    return new double[] {
      speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond
    };
  }

  public static Translation2d projectTo2d(Translation3d translation) {
    return new Translation2d(translation.getX(), translation.getZ());
  }
}
