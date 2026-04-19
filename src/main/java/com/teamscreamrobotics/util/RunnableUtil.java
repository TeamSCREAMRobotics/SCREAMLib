package com.teamscreamrobotics.util;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/** Thread-safe utility classes for executing runnables conditionally or exactly once. */
public class RunnableUtil {

  /**
   * Executes a {@link Runnable} at most once (or once per triggering condition).
   * All state is thread-safe via atomics.
   */
  public static class RunOnce {
    private final AtomicBoolean hasRun = new AtomicBoolean(false);
    private final AtomicReference<Object> lastValue = new AtomicReference<>();
    private final AtomicBoolean isInitialized = new AtomicBoolean(false);
    private final AtomicBoolean conditionMet = new AtomicBoolean(false);

    /** Runs {@code runnable} the first time this method is called; subsequent calls are no-ops. */
    public void runOnce(Runnable runnable) {
      if (hasRun.compareAndSet(false, true)) {
        runnable.run();
      }
    }

    /** Runs {@code runnable} the first time {@code condition} is {@code true}; no-op thereafter. */
    public void runOnceWhen(Runnable runnable, boolean condition) {
      if (!hasRun.get() && condition && hasRun.compareAndSet(false, true)) {
        runnable.run();
      }
    }

    /**
     * Runs {@code runnable} each time {@code newValue} differs from the previous call's value.
     * Does not fire on the first call (initialization).
     *
     * @param runnable the action to execute on change
     * @param newValue the value to compare against the previous call
     */
    public <T> void runOnceWhenChanged(Runnable runnable, T newValue) {
      if (isInitialized.get() && !newValue.equals(lastValue.get())) {
        lastValue.set(newValue);
        runnable.run();
      } else if (!isInitialized.getAndSet(true)) {
        lastValue.set(newValue);
      }
    }

    /**
     * Fires {@code runnable} the first time {@code condition} becomes {@code true}, then again
     * on every subsequent change to {@code newValue}.
     *
     * @param runnable  the action to execute
     * @param condition the trigger condition
     * @param newValue  the value to monitor for changes after the condition is met
     */
    public <T> void runOnceWhenTrueThenWhenChanged(
        Runnable runnable, boolean condition, T newValue) {
      if (!conditionMet.get()) {
        if (condition && conditionMet.compareAndSet(false, true)) {
          runnable.run();
          lastValue.set(newValue);
          isInitialized.set(true);
        }
      } else if (isInitialized.get() && !newValue.equals(lastValue.get())) {
        lastValue.set(newValue);
        runnable.run();
      }
    }

    /** Resets the run state so the next call to any {@code run*} method fires again. */
    public void reset() {
      hasRun.set(false);
    }
  }

  /**
   * Executes a {@link Runnable} until a stop condition is met, with optional retry-on-exception.
   */
  public static class RunUntil {
    private final AtomicBoolean shouldStop = new AtomicBoolean(false);

    /**
     * Runs {@code runnable} once if {@code stopCondition} is {@code true} and has not already stopped.
     * Marks as stopped after running.
     *
     * @param runnable      the action to execute
     * @param stopCondition when {@code true}, executes and then stops future calls
     */
    public void runUntil(Runnable runnable, boolean stopCondition) {
      if (!shouldStop.get() && stopCondition) {
        runnable.run();
        shouldStop.compareAndSet(false, true);
      }
    }

    /**
     * Retries {@code runnable} in a loop while {@code stopCondition} is {@code true} and
     * not yet stopped, silently swallowing exceptions until it succeeds.
     *
     * @param runnable      the action to attempt
     * @param stopCondition loop continues while this is {@code true} and not stopped
     */
    public void tryUntil(Runnable runnable, boolean stopCondition) {
      while (!shouldStop.get() && stopCondition) {
        try {
          runnable.run();
          break;
        } catch (Exception e) {
        }
      }
      shouldStop.compareAndSet(false, true);
    }

    /**
     * Retries {@code runnable} until it succeeds without throwing, silently swallowing exceptions.
     * Marks as stopped on first success.
     *
     * @param runnable the action to attempt repeatedly
     */
    public void tryUntil(Runnable runnable) {
      while (!shouldStop.get()) {
        try {
          runnable.run();
          break;
        } catch (Exception e) {
        }
      }
      shouldStop.compareAndSet(false, true);
    }

    /** Resets the stopped state so future calls to {@code runUntil}/{@code tryUntil} can fire again. */
    public void reset() {
      shouldStop.set(false);
    }

    /** Returns {@code true} if this instance has reached its stop condition. */
    public boolean hasStopped() {
      return shouldStop.get();
    }
  }
}
