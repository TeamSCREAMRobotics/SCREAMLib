package com.SCREAMLib.drivers;

import java.util.function.DoubleSupplier;

import com.SCREAMLib.drivers.TalonFXSubsystem.ControlType;

public interface TalonFXSubsystemGoal {
    DoubleSupplier target();
    ControlType controlType();
}