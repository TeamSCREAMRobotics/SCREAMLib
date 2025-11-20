package com.teamscreamrobotics.util;

import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class PPUtil {

  public static PIDConstants screamPIDConstantsToPPConstants(
      ScreamPIDConstants screamPIDConstants) {
    return new PIDConstants(
        screamPIDConstants.kP(),
        screamPIDConstants.kI(),
        screamPIDConstants.kD(),
        screamPIDConstants.integralZone());
  }

  public static Optional<PathPlannerPath> loadPathFile(String pathName) {
    try {
      return Optional.of(PathPlannerPath.fromPathFile(pathName));
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load path: " + pathName + " | " + e.getMessage(), e.getStackTrace());
      return Optional.empty();
    }
  }
}
