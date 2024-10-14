package com.SCREAMLib.util;

import com.SCREAMLib.pid.ScreamPIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;

public class PPUtil {

  public static PIDConstants screamPIDConstantsToPPConstants(
      ScreamPIDConstants screamPIDConstants) {
    return new PIDConstants(
        screamPIDConstants.kP(),
        screamPIDConstants.kI(),
        screamPIDConstants.kD(),
        screamPIDConstants.integralZone());
  }

  public static PPHolonomicDriveController createHolonomicDriveController(
      HolonomicPathFollowerConfig config) {
    return new PPHolonomicDriveController(
        config.translationConstants,
        config.rotationConstants,
        config.period,
        config.maxModuleSpeed,
        config.driveBaseRadius);
  }
}
