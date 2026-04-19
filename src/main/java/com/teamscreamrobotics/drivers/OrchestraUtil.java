package com.teamscreamrobotics.drivers;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

/** Singleton wrapper around a CTRE {@link Orchestra} for playing {@code .chrp} music files. */
public class OrchestraUtil {

  private static Orchestra orchestra = new Orchestra();

  /**
   * Registers one or more TalonFX motors as instruments in the orchestra.
   *
   * @param FXs the motors to add
   */
  public static void add(TalonFX... FXs) {
    for (TalonFX fx : FXs) {
      orchestra.addInstrument(fx);
    }
  }

  /**
   * Loads and plays a {@code .chrp} music file from the deploy directory.
   *
   * @param fileName the file name without the {@code .chrp} extension
   */
  public static void play(String fileName) {
    orchestra.loadMusic(fileName + ".chrp");
    orchestra.play();
  }

  /** Returns {@code true} if the orchestra is currently playing. */
  public static boolean isPlaying() {
    return orchestra.isPlaying();
  }

  /** Pauses playback without resetting the track position. */
  public static void pause() {
    orchestra.pause();
  }

  /** Stops playback and resets the track position to the beginning. */
  public static void stop() {
    orchestra.stop();
  }
}
