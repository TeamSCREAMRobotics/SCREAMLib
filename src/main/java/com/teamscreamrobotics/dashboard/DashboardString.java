package com.teamscreamrobotics.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** A String value backed by a NetworkTables entry for dashboard interaction. */
public class DashboardString {
  private final NetworkTableEntry entry;
  private String currentValue;

  /**
   * Creates a dashboard-linked string.
   *
   * @param tableName    NetworkTables table to publish under
   * @param key          entry key within the table
   * @param defaultValue initial value written to the table
   */
  public DashboardString(String tableName, String key, String defaultValue) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
    entry = table.getEntry(key);
    currentValue = defaultValue;
    entry.setDefaultString(defaultValue);
  }

  /** Pushes {@code value} to the dashboard and caches it locally. */
  public void set(String value) {
    currentValue = value;
    entry.setString(value);
  }

  /** Returns the current dashboard value, falling back to the last locally set value. */
  public String get() {
    return entry.getString(currentValue);
  }
}
