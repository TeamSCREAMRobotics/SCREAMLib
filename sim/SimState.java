package com.SCREAMLib.sim;

public record SimState(double position, double velocity, double supplyVoltage) {
  /**
   * Gets the position from the simulation.
   *
   * <p>The units for each simulation type are as follows:
   *
   * <ul>
   *   <li>DCMotorSim: Angular Position (Rotations)
   *   <li>ElevatorSim: Linear Position (Meters)
   *   <li>SingleJointedArmSim: Angular Position (Rotations)
   *   <li>FlywheelSim: N/A
   * </ul>
   */
  @Override
  public double position() {
    return position;
  }

  /**
   * Gets the velocity from the simulation.
   *
   * <p>The units for each simulation type are as follows:
   *
   * <ul>
   *   <li>DCMotorSim: Angular Velocity (rot/s)
   *   <li>ElevatorSim: Linear Velocity (m/s)
   *   <li>SingleJointedArmSim: Angular Velocity (rot/s)
   *   <li>FlywheelSim: Angular Velocity (rot/s)
   * </ul>
   */
  @Override
  public double velocity() {
    return velocity;
  }
}
