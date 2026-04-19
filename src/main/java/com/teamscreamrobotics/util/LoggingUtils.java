package com.teamscreamrobotics.util;

/** Shared logging utilities and marker interfaces. */
public class LoggingUtils {

    /** Implemented by subsystems or components that publish their own telemetry. */
    public interface Loggable {
        /** Publishes all relevant telemetry for this object. */
        public void log();
    }
}
