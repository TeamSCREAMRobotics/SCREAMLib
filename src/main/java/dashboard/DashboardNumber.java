package dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DashboardNumber {
  private final NetworkTableEntry entry;
  private double currentValue;

  public DashboardNumber(String tableName, String key, double defaultValue) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
    entry = table.getEntry(key);
    currentValue = defaultValue;
    entry.setDefaultDouble(defaultValue);
  }

  public void set(double value) {
    currentValue = value;
    entry.setDouble(value);
  }

  public double get() {
    return entry.getDouble(currentValue);
  }
}
