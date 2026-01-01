package dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DashboardString {
  private final NetworkTableEntry entry;
  private String currentValue;

  public DashboardString(String tableName, String key, String defaultValue) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
    entry = table.getEntry(key);
    currentValue = defaultValue;
    entry.setDefaultString(defaultValue);
  }

  public void set(String value) {
    currentValue = value;
    entry.setString(value);
  }

  public String get() {
    return entry.getString(currentValue);
  }
}
