package com.SCREAMLib.drivers;

import com.SCREAMLib.drivers.TalonFXSubsystem.ControlType;
import java.util.function.DoubleSupplier;

public interface TalonFXSubsystemGoal {
  DoubleSupplier target();

  ControlType controlType();
}
