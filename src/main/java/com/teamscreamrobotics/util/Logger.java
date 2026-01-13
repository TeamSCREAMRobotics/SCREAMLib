package com.teamscreamrobotics.util;

import dev.doglog.DogLog;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.Map;

public class Logger extends DogLog {
  private static boolean debug = false;
  private static final Map<String, Double> lastLogTimes = new HashMap<>();
  private static final double DEFAULT_LOG_FREQUENCY = 0.0;

  public static void enableDebug(boolean enable) {
    debug = enable;
  }

  private static boolean shouldLog(String key, double frequencySeconds) {
    if (frequencySeconds <= 0.0) {
      return true;
    }

    double currentTime = Timer.getFPGATimestamp();
    double lastLogTime = lastLogTimes.getOrDefault(key, 0.0);
    double elapsedTime = currentTime - lastLogTime;

    if (elapsedTime >= frequencySeconds) {
      lastLogTimes.put(key, currentTime);
      return true;
    }
    return false;
  }

  public static void log(String key, Mechanism2d value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      SmartDashboard.putData(key, value);
    }
  }

  public static void log(String key, Mechanism2d value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Boolean
  public static void log(String key, boolean value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static void log(String key, boolean value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Double
  public static void log(String key, double value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static void log(String key, double value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Float
  public static void log(String key, float value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static void log(String key, float value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Integer
  public static void log(String key, int value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static void log(String key, int value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Long
  public static void log(String key, long value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static void log(String key, long value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // String
  public static void log(String key, String value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static void log(String key, String value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Enum
  public static <E extends Enum<E>> void log(String key, E value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static <E extends Enum<E>> void log(String key, E value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // StructSerializable
  public static <T extends StructSerializable> void log(
      String key, T value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static <T extends StructSerializable> void log(String key, T value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // ==== STANDARD LOGGING METHODS (ARRAYS) ====

  // Boolean array
  public static void log(String key, boolean[] value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static void log(String key, boolean[] value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Double array
  public static void log(String key, double[] value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static void log(String key, double[] value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Float array
  public static void log(String key, float[] value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static void log(String key, float[] value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Integer array
  public static void log(String key, int[] value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static void log(String key, int[] value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Long array
  public static void log(String key, long[] value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static void log(String key, long[] value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // String array
  public static void log(String key, String[] value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static void log(String key, String[] value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Enum array
  public static <E extends Enum<E>> void log(String key, E[] value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static <E extends Enum<E>> void log(String key, E[] value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // StructSerializable array
  public static <T extends StructSerializable> void log(
      String key, T[] value, double frequencySeconds) {
    if (shouldLog(key, frequencySeconds)) {
      DogLog.log(key, value);
    }
  }

  public static <T extends StructSerializable> void log(String key, T[] value) {
    log(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // ==== DEBUG LOGGING METHODS (SINGLE VALUES) ====

  // Mechanism2d
  public static void logDebug(String key, Mechanism2d value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      SmartDashboard.putData(key, value);
    }
  }

  public static void logDebug(String key, Mechanism2d value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Boolean
  public static void logDebug(String key, boolean value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static void logDebug(String key, boolean value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Double
  public static void logDebug(String key, double value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static void logDebug(String key, double value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Float
  public static void logDebug(String key, float value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static void logDebug(String key, float value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Integer
  public static void logDebug(String key, int value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static void logDebug(String key, int value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Long
  public static void logDebug(String key, long value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static void logDebug(String key, long value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // String
  public static void logDebug(String key, String value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static void logDebug(String key, String value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Enum
  public static <E extends Enum<E>> void logDebug(String key, E value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static <E extends Enum<E>> void logDebug(String key, E value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // StructSerializable
  public static <T extends StructSerializable> void logDebug(
      String key, T value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static <T extends StructSerializable> void logDebug(String key, T value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // ==== DEBUG LOGGING METHODS (ARRAYS) ====

  // Boolean array
  public static void logDebug(String key, boolean[] value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static void logDebug(String key, boolean[] value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Double array
  public static void logDebug(String key, double[] value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static void logDebug(String key, double[] value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Float array
  public static void logDebug(String key, float[] value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static void logDebug(String key, float[] value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Integer array
  public static void logDebug(String key, int[] value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static void logDebug(String key, int[] value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Long array
  public static void logDebug(String key, long[] value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static void logDebug(String key, long[] value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // String array
  public static void logDebug(String key, String[] value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static void logDebug(String key, String[] value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // Enum array
  public static <E extends Enum<E>> void logDebug(String key, E[] value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static <E extends Enum<E>> void logDebug(String key, E[] value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }

  // StructSerializable array
  public static <T extends StructSerializable> void logDebug(
      String key, T[] value, double frequencySeconds) {
    if (debug && shouldLog(key, frequencySeconds)) {
      log(key, value);
    }
  }

  public static <T extends StructSerializable> void logDebug(String key, T[] value) {
    logDebug(key, value, DEFAULT_LOG_FREQUENCY);
  }
}