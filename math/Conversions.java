/*
 * Initially from https://github.com/Team364/BaseFalconSwerve
 */

package com.team4522.lib.math;

import com.team4522.lib.data.Length;

public class Conversions {
  /**
   * @param rps Falcon rotations per second
   * @param gearRatio gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
   * @return RPM of mechanism
   */
  public static double rpsToRPM(double falconRPS, double gearRatio) {
    double motorRPM = falconRPS * 60.0;
    return motorRPM / gearRatio;
  }

  /**
   * @param RPM RPM of mechanism
   * @param gearRatio Gear ratio between Falcon and mechanism (set to 1 for Falcon RPS)
   * @return Falcon rotations per second
   */
  public static double rpmToRPS(double rpm, double gearRatio) {
    double motorRPM = rpm * gearRatio;
    return motorRPM / 60.0;
  }

  /**
   * @param falconRPS Falcon rotations per second
   * @param circumference circumference of wheel in meters
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return mechanism linear velocity in meters per second
   */
  public static double rpsToMPS(
      double falconRPS, double circumference, double gearRatio) {
    double wheelRPM = rpsToRPM(falconRPS, gearRatio);
    return (wheelRPM * circumference) / 60.0;
  }

  /**
   * @param velocity velocity in meters per second
   * @param circumference circumference of wheel in meters
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return Falcon rotations per second
   */
  public static double mpsToRPS(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    return rpmToRPS(wheelRPM, gearRatio);
  }

  public static double rpmToFTS(double rpm, double circumference){
    return (circumference * rpm) / 60;
  }

  public static double linearDistanceToRotations(Length distance, Length circumference){
        return distance.getInches() / circumference.getInches();
  }

  public static Length rotationsToLinearDistance(double rotations, Length circumference){
      return circumference.times(rotations);
  }
}