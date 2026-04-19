package com.teamscreamrobotics.util;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Convenience factory methods for creating WPILib simulation objects. */
public class SimUtil {

  /**
   * Creates a {@link DCMotorSim} for the given gearbox and moment of inertia.
   *
   * @param gearbox             the motor gearbox model
   * @param gearing             gear ratio (input/output)
   * @param JKgMetersSquaredMOI moment of inertia at the mechanism in kg·m²
   */
  public static DCMotorSim createDCMotorSim(
      DCMotor gearbox, double gearing, double JKgMetersSquaredMOI) {
    return new DCMotorSim(
        LinearSystemId.createDCMotorSystem(gearbox, JKgMetersSquaredMOI, gearing), gearbox);
  }

  /**
   * Creates a {@link DCMotorSim} with measurement noise.
   *
   * @param gearbox             the motor gearbox model
   * @param gearing             gear ratio (input/output)
   * @param JKgMetersSquaredMOI moment of inertia at the mechanism in kg·m²
   * @param positionStdDev      standard deviation of position measurement noise (rotations)
   * @param velocityStdDev      standard deviation of velocity measurement noise (RPS)
   */
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

  /**
   * Creates a {@link FlywheelSim} for the given gearbox and moment of inertia.
   *
   * @param gearbox             the motor gearbox model
   * @param gearing             gear ratio (input/output)
   * @param JKgMetersSquaredMOI moment of inertia at the flywheel in kg·m²
   */
  public static FlywheelSim createFlywheelSim(
      DCMotor gearbox, double gearing, double JKgMetersSquaredMOI) {
    return new FlywheelSim(
        LinearSystemId.createFlywheelSystem(gearbox, JKgMetersSquaredMOI, gearing), gearbox);
  }

  /**
   * Creates a {@link FlywheelSim} with measurement noise.
   *
   * @param gearbox             the motor gearbox model
   * @param gearing             gear ratio (input/output)
   * @param JKgMetersSquaredMOI moment of inertia at the flywheel in kg·m²
   * @param positionStdDev      standard deviation of position measurement noise (rotations)
   * @param velocityStdDev      standard deviation of velocity measurement noise (RPS)
   */
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
