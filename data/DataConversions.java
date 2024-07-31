package com.team4522.lib.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DataConversions {
    
    public static double[] pose2dToArray(Pose2d pose){
        return new double[]{pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
    }

    public static double[] translation3dToArray(Translation3d translation){
        return new double[]{translation.getX(), translation.getY(), translation.getZ(), 0, 0, 0, 0};
    }

    public static double[] translation3dToArray(Translation3d translation, Rotation3d rot){
        Quaternion quat = rot.getQuaternion();
        return new double[]{translation.getX(), translation.getY(), translation.getZ(), quat.getX(), quat.getY(), quat.getZ(), quat.getW()};
    }

    public static Translation3d pose3dArrayToTranslation3d(double[] array){
        return new Translation3d(array[0], array[1], array[2]);
    }

    public static double[] translation3dArrayToNumArray(Translation3d[] translations){
        double[] combinedArr = new double[translations.length * 7];
        int index = 0;

        for (Translation3d translation : translations) {
            double[] arr = translation3dToArray(translation);

            System.arraycopy(arr, 0, combinedArr, index, arr.length);

            index += arr.length;
        }

        return combinedArr;
    }

    public static double[] translation2dArrayToNumArray(Translation2d[] translations){
        double[] combinedArr = new double[translations.length * 7];
        int index = 0;

        for (Translation2d translation : translations) {
            double[] arr = translation3dToArray(new Translation3d(translation.getX(), translation.getY(), 0.0));

            System.arraycopy(arr, 0, combinedArr, index, arr.length);

            index += arr.length;
        }

        return combinedArr;
    }

    public static double[] translation2dArrayToNumArray(Translation2d[] translations, double z){
        double[] combinedArr = new double[translations.length * 7];
        int index = 0;

        for (Translation2d translation : translations) {
            double[] arr = translation3dToArray(new Translation3d(translation.getX(), translation.getY(), z));

            System.arraycopy(arr, 0, combinedArr, index, arr.length);

            index += arr.length;
        }

        return combinedArr;
    }

    public static Translation2d chassisSpeedsToTranslation(ChassisSpeeds speeds){
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }
}
