package com.teamscreamrobotics.vision;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.TimestampedDoubleArray;

/**
 * Reads Limelight data from NetworkTables. All NT reads are captured atomically so the
 * server-side timestamp can be stored alongside the array for latency-compensated replay.
 */
public class LimelightIOReal implements LimelightIO {

    private final String name;

    public LimelightIOReal(Limelight limelight) {
        this.name = limelight.name();
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        DoubleArrayEntry mt1Entry = LimelightHelpers.getLimelightDoubleArrayEntry(name, "botpose_wpiblue");
        TimestampedDoubleArray mt1 = mt1Entry.getAtomic();
        inputs.botpose_wpiblue = mt1.value;
        inputs.timestampSeconds = mt1.timestamp / 1_000_000.0;

        DoubleArrayEntry mt2Entry = LimelightHelpers.getLimelightDoubleArrayEntry(name, "botpose_orb_wpiblue");
        TimestampedDoubleArray mt2 = mt2Entry.getAtomic();
        inputs.botpose_orb_wpiblue = mt2.value;
        inputs.orbTimestampSeconds = mt2.timestamp / 1_000_000.0;

        inputs.tv = LimelightHelpers.getTV(name);
        inputs.tl = LimelightHelpers.getLimelightNTDouble(name, "tl");
        inputs.cl = LimelightHelpers.getLimelightNTDouble(name, "cl");
        inputs.rawFiducials = LimelightHelpers.getLimelightNTDoubleArray(name, "rawfiducials");
    }
}
