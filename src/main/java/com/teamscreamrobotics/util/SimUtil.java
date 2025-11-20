package com.teamscreamrobotics.util;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SimUtil {

  public static DCMotorSim createDCMotorSim(
      DCMotor gearbox, double gearing, double JKgMetersSquaredMOI) {
    return new DCMotorSim(
        LinearSystemId.createDCMotorSystem(gearbox, JKgMetersSquaredMOI, gearing), gearbox);
  }

  public static DCMotorSim createDCMotorSim(
      DCMotor gearbox,
      double gearing,
      double JKgMetersSquaredMOI,
      double positionStdDev,
      double velocityStdDev) {
    return new DCMotorSim(
        LinearSystemId.createDCMotorSystem(gearbox, JKgMetersSquaredMOI, gearing),
        gearbox,
        positionStdDev,
        velocityStdDev);
  }

  public static FlywheelSim createFlywheelSim(
      DCMotor gearbox, double gearing, double JKgMetersSquaredMOI) {
    return new FlywheelSim(
        LinearSystemId.createFlywheelSystem(gearbox, JKgMetersSquaredMOI, gearing), gearbox);
  }

  public static FlywheelSim createFlywheelSim(
      DCMotor gearbox,
      double gearing,
      double JKgMetersSquaredMOI,
      double positionStdDev,
      double velocityStdDev) {
    return new FlywheelSim(
        LinearSystemId.createFlywheelSystem(gearbox, JKgMetersSquaredMOI, gearing),
        gearbox,
        positionStdDev,
        velocityStdDev);
  }
}
