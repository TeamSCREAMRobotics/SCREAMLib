package com.teamscreamrobotics.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** A boolean value backed by a NetworkTables entry for dashboard interaction. */
public class DashboardBoolean {
  private final NetworkTableEntry entry;
  private boolean currentValue;

  /**
   * Creates a dashboard-linked boolean.
   *
   * @param tableName    NetworkTables table to publish under
   * @param key          entry key within the table
   * @param defaultValue initial value written to the table
   */
  public DashboardBoolean(String tableName, String key, boolean defaultValue) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
    entry = table.getEntry(key);
    currentValue = defaultValue;
    entry.setDefaultBoolean(defaultValue);
  }

  /** Pushes {@code value} to the dashboard and caches it locally. */
  public void set(boolean value) {
    currentValue = value;
    entry.setBoolean(value);
  }

  /** Returns the current dashboard value, falling back to the last locally set value. */
  public boolean get() {
    return entry.getBoolean(currentValue);
  }
}
