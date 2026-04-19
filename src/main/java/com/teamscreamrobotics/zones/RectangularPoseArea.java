package com.teamscreamrobotics.zones;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** An axis-aligned rectangular 2D zone on the field used for pose containment checks. */
public class RectangularPoseArea {
  private final Translation2d bottomLeft;
  private final Translation2d topRight;

  // From FRC 6391
  // https://github.com/6391-Ursuline-Bearbotics/2024-6391-Crescendo/blob/master/src/main/java/frc/robot/Util/RectanglePoseArea.java

  /**
   * Create a 2D rectangular area for pose calculations.
   *
   * @param bottomLeft bottom left corner of the rectangle.
   * @param topRight top right corner of the rectangle.
   */
  public RectangularPoseArea(Translation2d bottomLeft, Translation2d topRight) {
    this.bottomLeft = bottomLeft;
    this.topRight = topRight;
  }

  /** Returns the minimum X coordinate (left edge) in meters. */
  public double getMinX() {
    return bottomLeft.getX();
  }

  /** Returns the maximum X coordinate (right edge) in meters. */
  public double getMaxX() {
    return topRight.getX();
  }

  /** Returns the minimum Y coordinate (bottom edge) in meters. */
  public double getMinY() {
    return bottomLeft.getY();
  }

  /** Returns the maximum Y coordinate (top edge) in meters. */
  public double getMaxY() {
    return topRight.getY();
  }

  /** Returns the bottom-left corner of the rectangle. */
  public Translation2d getBottomLeftPoint() {
    return bottomLeft;
  }

  /** Returns the top-right corner of the rectangle. */
  public Translation2d getTopRightPoint() {
    return topRight;
  }

  /** Returns {@code true} if the pose's translation is within the rectangle. */
  public boolean contains(Pose2d pose) {
    return contains(pose.getTranslation());
  }

  /** Returns {@code true} if the translation is within the rectangle (inclusive bounds). */
  public boolean contains(Translation2d pose) {
    return pose.getX() >= bottomLeft.getX()
        && pose.getX() <= topRight.getX()
        && pose.getY() >= bottomLeft.getY()
        && pose.getY() <= topRight.getY();
  }
}
