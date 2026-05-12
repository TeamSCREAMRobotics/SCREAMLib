package com.teamscreamrobotics.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

// Manually implemented equivalent of what @AutoLog would generate for LimelightIOInputs.
public class LimelightIOInputsAutoLogged extends LimelightIO.LimelightIOInputs implements LoggableInputs {

    @Override
    public void toLog(LogTable table) {
        table.put("BotposeWpiblue", botpose_wpiblue);
        table.put("BotposeOrbWpiblue", botpose_orb_wpiblue);
        table.put("TimestampSeconds", timestampSeconds);
        table.put("OrbTimestampSeconds", orbTimestampSeconds);
        table.put("Tv", tv);
        table.put("Tl", tl);
        table.put("Cl", cl);
        table.put("RawFiducials", rawFiducials);
    }

    @Override
    public void fromLog(LogTable table) {
        botpose_wpiblue = table.get("BotposeWpiblue", botpose_wpiblue);
        botpose_orb_wpiblue = table.get("BotposeOrbWpiblue", botpose_orb_wpiblue);
        timestampSeconds = table.get("TimestampSeconds", timestampSeconds);
        orbTimestampSeconds = table.get("OrbTimestampSeconds", orbTimestampSeconds);
        tv = table.get("Tv", tv);
        tl = table.get("Tl", tl);
        cl = table.get("Cl", cl);
        rawFiducials = table.get("RawFiducials", rawFiducials);
    }
}
