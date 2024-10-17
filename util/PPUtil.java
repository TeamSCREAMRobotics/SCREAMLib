package com.SCREAMLib.util;

import com.SCREAMLib.pid.ScreamPIDConstants;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;

public class PPUtil {

  public static PIDConstants screamPIDConstantsToPPConstants(
      ScreamPIDConstants screamPIDConstants) {
    return new PIDConstants(
        screamPIDConstants.kP(),
        screamPIDConstants.kI(),
        screamPIDConstants.kD(),
        screamPIDConstants.integralZone());
  }

  public static PathPlannerPath loadPathFile(String pathName) {
    try {
      return PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load path: " + pathName + " | " + e.getMessage(), e.getStackTrace());
      return PathPlannerPath.fromPathPoints(
          List.of(), new PathConstraints(0, 0, 0, 0), new GoalEndState(0, Rotation2d.kZero));
    }
  }
}
