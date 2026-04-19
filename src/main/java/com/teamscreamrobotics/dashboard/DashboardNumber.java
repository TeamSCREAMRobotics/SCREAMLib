package com.teamscreamrobotics.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** A double value backed by a NetworkTables entry for dashboard interaction. */
public class DashboardNumber {
  private final NetworkTableEntry entry;
  private double currentValue;

  /**
   * Creates a dashboard-linked number.
   *
   * @param tableName    NetworkTables table to publish under
   * @param key          entry key within the table
   * @param defaultValue initial value written to the table
   */
  public DashboardNumber(String tableName, String key, double defaultValue) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
    entry = table.getEntry(key);
    currentValue = defaultValue;
    entry.setDefaultDouble(defaultValue);
  }

  /** Pushes {@code value} to the dashboard and caches it locally. */
  public void set(double value) {
    currentValue = value;
    entry.setDouble(value);
  }

  /** Returns the current dashboard value, falling back to the last locally set value. */
  public double get() {
    return entry.getDouble(currentValue);
  }
}
