package com.SCREAMLib.zones;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public class PolygonalPoseArea {

  private final List<Translation2d> vertices;

  public PolygonalPoseArea(List<Translation2d> vertices) {
    this.vertices = vertices;
  }

  public boolean contains(Pose2d pose) {
    return contains(pose.getTranslation());
  }

  public boolean contains(Translation2d point) {
    int n = vertices.size();
    boolean result = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
      if ((vertices.get(i).getY() > point.getY()) != (vertices.get(j).getY() > point.getY())
          && (point.getX()
              < (vertices.get(j).getX() - vertices.get(i).getX())
                      * (point.getY() - vertices.get(i).getY())
                      / (vertices.get(j).getY() - vertices.get(i).getY())
                  + vertices.get(i).getX())) {
        result = !result;
      }
    }
    return result;
  }

  public Translation2d getCenter() {
    double centerX = 0.0;
    double centerY = 0.0;
    double signedArea = 0.0;
    double currentX;
    double currentY;
    double nextX;
    double nextY;
    double a;

    int n = vertices.size();
    for (int i = 0; i < n; i++) {
      currentX = vertices.get(i).getX();
      currentY = vertices.get(i).getY();
      nextX = vertices.get((i + 1) % n).getX();
      nextY = vertices.get((i + 1) % n).getY();
      a = currentX * nextY - nextX * currentY;
      signedArea += a;
      centerX += (currentX + nextX) * a;
      centerY += (currentY + nextY) * a;
    }

    signedArea *= 0.5;
    centerX /= (6.0 * signedArea);
    centerY /= (6.0 * signedArea);

    return new Translation2d(centerX, centerY);
  }
}
