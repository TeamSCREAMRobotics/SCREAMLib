package com.teamscreamrobotics.vision;

import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {

    @AutoLog
    class LimelightIOInputs {
        /**
         * Full NT array from {@code botpose_wpiblue}:
         * [x, y, z, roll, pitch, yaw, latencyMs, tagCount, tagSpan, avgDist, avgArea,
         *  <7 values per tag: id, txnc, tync, ta, distToCamera, distToRobot, ambiguity>]
         */
        public double[] botpose_wpiblue = new double[0];
        /** Same layout as botpose_wpiblue, from {@code botpose_orb_wpiblue} (MegaTag2). */
        public double[] botpose_orb_wpiblue = new double[0];
        /**
         * NT server timestamp for botpose_wpiblue, in seconds (converted from microseconds).
         * Used to reconstruct the latency-compensated measurement timestamp during replay.
         */
        public double timestampSeconds = 0.0;
        /** NT server timestamp for botpose_orb_wpiblue, in seconds. */
        public double orbTimestampSeconds = 0.0;
        public boolean tv = false;
        public double tl = 0.0;
        public double cl = 0.0;
        /** Raw fiducial data from the {@code rawfiducials} NT entry (7 doubles per tag). */
        public double[] rawFiducials = new double[0];
    }

    void updateInputs(LimelightIOInputs inputs);
}
