package com.teamscreamrobotics.zones;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** A circular 2D zone on the field used for pose containment checks. */
public class CircularPoseArea {

  private final Translation2d center;
  private final double diameter;

  /**
   * Creates a circular zone.
   *
   * @param center   center of the circle in field coordinates (meters)
   * @param diameter diameter of the circle in meters
   */
  public CircularPoseArea(Translation2d center, double diameter) {
    this.center = center;
    this.diameter = diameter;
  }

  /** Returns the center of the circle in field coordinates. */
  public Translation2d getCenter() {
    return center;
  }

  /** Returns the diameter of the circle in meters. */
  public double getDiameter() {
    return diameter;
  }

  /** Returns {@code true} if the pose's translation is inside the circle. */
  public boolean contains(Pose2d pose) {
    return contains(pose.getTranslation());
  }

  /** Returns {@code true} if the translation is inside the circle. */
  public boolean contains(Translation2d translation) {
    return center.getDistance(translation) <= (diameter / 2.0);
  }
}
