package com.team4522.lib.util;

import java.util.concurrent.atomic.AtomicBoolean;

public class RunnableUtil {

    public static class RunOnce {
        private final AtomicBoolean hasRun = new AtomicBoolean(false);

        public void runOnce(Runnable runnable) {
            if (hasRun.compareAndSet(false, true)) {
                runnable.run();
            }
        }

        public void runOnceWhen(Runnable runnable, boolean condition) {
            if (!hasRun.get() && condition && hasRun.compareAndSet(false, true)) {
                runnable.run();
            }
        }

        public void reset() {
            hasRun.set(false);
        }
    }

    public static class RunUntil {
        private final AtomicBoolean shouldStop = new AtomicBoolean(false);
    
        public void runUntil(Runnable runnable, boolean stopCondition) {
            if (!shouldStop.get() && stopCondition) {
                runnable.run();
                shouldStop.compareAndSet(false, true);
            }
        }

        public void tryUntil(Runnable runnable, boolean stopCondition) {
            while (!shouldStop.get() && stopCondition) {
                try {
                    runnable.run();
                    break;
                } catch (Exception e) {}
            }
            shouldStop.compareAndSet(false, true);
        }

        public void tryUntil(Runnable runnable) {
            while (!shouldStop.get()) {
                try {
                    runnable.run();
                    break;
                } catch (Exception e) {}
            }
            shouldStop.compareAndSet(false, true);
        }
    
        public void reset() {
            shouldStop.set(false);
        }
    
        public boolean hasStopped(){
            return shouldStop.get();
        }
    }
}
