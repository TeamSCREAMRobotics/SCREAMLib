package com.teamscreamrobotics.zones;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class CircularPoseArea {

  private final Translation2d center;
  private final double diameter;

  public CircularPoseArea(Translation2d center, double diameter) {
    this.center = center;
    this.diameter = diameter;
  }

  public Translation2d getCenter() {
    return center;
  }

  public double getDiameter() {
    return diameter;
  }

  public boolean contains(Pose2d pose) {
    return contains(pose.getTranslation());
  }

  public boolean contains(Translation2d translation) {
    return center.getDistance(translation) <= (diameter / 2.0);
  }
}
