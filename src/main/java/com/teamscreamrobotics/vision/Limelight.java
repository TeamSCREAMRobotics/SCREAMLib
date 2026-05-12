package com.teamscreamrobotics.vision;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Describes one Limelight camera in a multi-camera vision system.
 *
 * @param name          NetworkTables table name, e.g. {@code "limelight-front"}
 * @param ip            Static IP for port-forwarding, e.g. {@code "10.TE.AM.11"}
 * @param robotToCamera Transform from robot center to camera lens in robot space
 */
public record Limelight(String name, String ip, Pose3d robotToCamera) {}
