package com.team4522.lib.drivers;

import java.util.function.DoubleSupplier;

import com.team4522.lib.drivers.TalonFXSubsystem.ControlType;

public interface TalonFXSubsystemGoal {
    DoubleSupplier target();
    ControlType controlType();
}