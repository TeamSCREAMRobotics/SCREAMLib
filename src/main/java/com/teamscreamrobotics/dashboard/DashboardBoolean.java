package com.teamscreamrobotics.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DashboardBoolean {
  private final NetworkTableEntry entry;
  private boolean currentValue;

  public DashboardBoolean(String tableName, String key, boolean defaultValue) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
    entry = table.getEntry(key);
    currentValue = defaultValue;
    entry.setDefaultBoolean(defaultValue);
  }

  public void set(boolean value) {
    currentValue = value;
    entry.setBoolean(value);
  }

  public boolean get() {
    return entry.getBoolean(currentValue);
  }
}
