package com.teamscreamrobotics.util;

import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

/** Utilities for integrating {@link ScreamPIDConstants} and PathPlanner. */
public class PPUtil {

  /**
   * Converts a {@link ScreamPIDConstants} to PathPlanner's {@link PIDConstants}.
   *
   * @param screamPIDConstants the source constants
   */
  public static PIDConstants screamPIDConstantsToPPConstants(
      ScreamPIDConstants screamPIDConstants) {
    return new PIDConstants(
        screamPIDConstants.kP(),
        screamPIDConstants.kI(),
        screamPIDConstants.kD(),
        screamPIDConstants.integralZone());
  }

  /**
   * Loads a PathPlanner path file, returning {@link Optional#empty()} and logging a DS error on failure.
   *
   * @param pathName the path name (no file extension)
   */
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
