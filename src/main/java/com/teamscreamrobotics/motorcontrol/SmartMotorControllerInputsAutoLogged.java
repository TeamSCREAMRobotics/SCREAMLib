package com.teamscreamrobotics.motorcontrol;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

// Manually implemented equivalent of what @AutoLog would generate for SmartMotorControllerInputs.
public class SmartMotorControllerInputsAutoLogged
        extends SmartMotorController.SmartMotorControllerInputs
        implements LoggableInputs {

    @Override
    public void toLog(LogTable table) {
        table.put("PositionRad", positionRad);
        table.put("VelocityRadPerSec", velocityRadPerSec);
        table.put("AppliedVolts", appliedVolts);
        table.put("SupplyCurrentAmps", supplyCurrentAmps);
        table.put("StatorCurrentAmps", statorCurrentAmps);
        table.put("TempCelsius", tempCelsius);
        table.put("Connected", connected);
        table.put("FollowerSupplyCurrentAmps", followerSupplyCurrentAmps);
        table.put("FollowerStatorCurrentAmps", followerStatorCurrentAmps);
        table.put("FollowerTempCelsius", followerTempCelsius);
        table.put("FollowerConnected", followerConnected);
    }

    @Override
    public void fromLog(LogTable table) {
        positionRad = table.get("PositionRad", positionRad);
        velocityRadPerSec = table.get("VelocityRadPerSec", velocityRadPerSec);
        appliedVolts = table.get("AppliedVolts", appliedVolts);
        supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
        statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
        tempCelsius = table.get("TempCelsius", tempCelsius);
        connected = table.get("Connected", connected);
        followerSupplyCurrentAmps = table.get("FollowerSupplyCurrentAmps", followerSupplyCurrentAmps);
        followerStatorCurrentAmps = table.get("FollowerStatorCurrentAmps", followerStatorCurrentAmps);
        followerTempCelsius = table.get("FollowerTempCelsius", followerTempCelsius);
        followerConnected = table.get("FollowerConnected", followerConnected);
    }
}
