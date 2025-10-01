// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.utils;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import java.util.HashMap;
import java.util.Map;

/** FlytDashboardV2 - Pub/Sub-based NetworkTables wrapper */
public class FlytDashboardV2 {

  private final NetworkTable table;
  private final Map<String, BooleanPublisher> boolPublishers = new HashMap<>();
  private final Map<String, IntegerPublisher> intPublishers = new HashMap<>();
  private final Map<String, DoublePublisher> doublePublishers = new HashMap<>();
  private final Map<String, StringPublisher> stringPublishers = new HashMap<>();

  private final Map<String, BooleanSubscriber> boolSubscribers = new HashMap<>();
  private final Map<String, IntegerSubscriber> intSubscribers = new HashMap<>();
  private final Map<String, DoubleSubscriber> doubleSubscribers = new HashMap<>();
  private final Map<String, StringSubscriber> stringSubscribers = new HashMap<>();

  public FlytDashboardV2(String tableName) {
    this.table = NetworkTableInstance.getDefault().getTable(tableName);
  }

  // === Boolean ===
  public void putBoolean(String key, boolean value) {
    boolPublishers.computeIfAbsent(key, k -> table.getBooleanTopic(k).publish()).set(value);
  }

  public boolean getBoolean(String key, boolean defaultValue) {
    return boolSubscribers
        .computeIfAbsent(key, k -> table.getBooleanTopic(k).subscribe(defaultValue))
        .get();
  }

  // === Double ===
  public void putDouble(String key, double value) {
    doublePublishers.computeIfAbsent(key, k -> table.getDoubleTopic(k).publish()).set(value);
  }

  public double getDouble(String key, double defaultValue) {
    return doubleSubscribers
        .computeIfAbsent(key, k -> table.getDoubleTopic(k).subscribe(defaultValue))
        .get();
  }

  // === Integer ===
  public void putInteger(String key, int value) {
    intPublishers.computeIfAbsent(key, k -> table.getIntegerTopic(k).publish()).set(value);
  }

  public int getInteger(String key, int defaultValue) {
    return (int)
        intSubscribers
            .computeIfAbsent(key, k -> table.getIntegerTopic(k).subscribe(defaultValue))
            .get();
  }

  // === String ===
  public void putString(String key, String value) {
    stringPublishers.computeIfAbsent(key, k -> table.getStringTopic(k).publish()).set(value);
  }

  public String getString(String key, String defaultValue) {
    return stringSubscribers
        .computeIfAbsent(key, k -> table.getStringTopic(k).subscribe(defaultValue))
        .get();
  }

  // === Bulk Logging ===
  public void putAll(Map<String, Object> values) {
    for (Map.Entry<String, Object> entry : values.entrySet()) {
      String key = entry.getKey();
      Object value = entry.getValue();
      if (value instanceof Boolean b) {
        putBoolean(key, b);
      } else if (value instanceof Integer i) {
        putInteger(key, i);
      } else if (value instanceof Double d) {
        putDouble(key, d);
      } else if (value instanceof String s) {
        putString(key, s);
      } else {
        System.out.println("[FlytDashboardV2] Unsupported type for key: " + key + " -> " + value);
      }
    }
  }
}
