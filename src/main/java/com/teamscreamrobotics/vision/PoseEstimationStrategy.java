package com.teamscreamrobotics.vision;

public enum PoseEstimationStrategy {
    /** Standard multi-tag solve using {@code getBotPoseEstimate_wpiBlue}. */
    MEGATAG_1,
    /** Gyro-assisted solve using {@code getBotPoseEstimate_wpiBlue_MegaTag2}. Locks heading to robot orientation. */
    MEGATAG_2
}
