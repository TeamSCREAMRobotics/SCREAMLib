package com.teamscreamrobotics.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.BooleanSupplier;

/** Utility for flipping or mirroring field-relative geometry between blue and red alliance. */
public final class AllianceFlipUtil {

  /** Full field dimensions (length × width) in meters. */
  public static final Translation2d FIELD_DIMENSIONS = new Translation2d(16.541, 8.211);
  /** Field width (Y axis) in meters. */
  public static final double FIELD_WIDTH = FIELD_DIMENSIONS.getY();
  /** Field length (X axis) in meters. */
  public static final double FIELD_LENGTH = FIELD_DIMENSIONS.getX();

  /** Returns a supplier that is {@code true} when the robot is on the red alliance. */
  public static BooleanSupplier shouldFlip() {
    return () -> DriverStation.getAlliance().filter(value -> value == Alliance.Red).isPresent();
  }

  /** Returns {@code 0°} for blue alliance and {@code 180°} for red — the forward-facing heading. */
  public static Rotation2d getFwdHeading() {
    return get(Rotation2d.kZero, Rotation2d.k180deg);
  }

  /** Returns {@code +1} for blue and {@code -1} for red — useful for signed field-relative math. */
  public static int getDirectionCoefficient() {
    return (int) get(1, -1);
  }

  /** Returns {@code blueValue} on blue alliance and {@code redValue} on red. */
  public static <T> T get(T blueValue, T redValue) {
    return get(blueValue, redValue, false);
  }

  /**
   * Returns the alliance-appropriate value, optionally inverted.
   *
   * @param blueValue the value to return for blue (or red when {@code inverse} is true)
   * @param redValue  the value to return for red (or blue when {@code inverse} is true)
   * @param inverse   if {@code true}, swaps which alliance gets which value
   */
  public static <T> T get(T blueValue, T redValue, boolean inverse) {
    return (inverse && !shouldFlip().getAsBoolean() || !inverse && shouldFlip().getAsBoolean())
        ? redValue
        : blueValue;
  }

  /** Returns the blue rotation, or blue + 180° on red (rotationally-symmetric flip). */
  public static Rotation2d FlippedRotation2d(Rotation2d blueValue) {
    return get(blueValue, blueValue.plus(Rotation2d.k180deg));
  }

  /** Returns the blue rotation, or the Z-flipped equivalent on red (rotationally-symmetric flip). */
  public static Rotation3d FlippedRotation3d(Rotation3d blueValue) {
    return new Rotation3d(
        blueValue.getX(),
        blueValue.getY(),
        FlippedRotation2d(Rotation2d.fromRadians(blueValue.getZ())).getRadians());
  }

  /** Returns the blue translation, or {@code (fieldLength - x, fieldWidth - y)} on red. */
  public static Translation2d FlippedTranslation2d(Translation2d blueValue) {
    return get(
        blueValue,
        new Translation2d(
            FIELD_LENGTH - blueValue.getX(),
            FIELD_WIDTH - blueValue.getY()));
  }

  /** Returns the blue pose, or the rotationally-flipped equivalent on red. */
  public static Pose2d FlippedPose2d(Pose2d blueValue) {
    return new Pose2d(
        FlippedTranslation2d(blueValue.getTranslation()),
        FlippedRotation2d(blueValue.getRotation()));
  }

  /** Returns the blue translation3d, or {@code (fieldLength - x, fieldWidth - y, z)} on red. */
  public static Translation3d FlippedTranslation3d(Translation3d blueValue) {
    return get(
        blueValue,
        new Translation3d(
            FIELD_LENGTH - blueValue.getX(),
            FIELD_WIDTH - blueValue.getY(),
            blueValue.getZ()));
  }

  /** Returns the blue pose3d, or the rotationally-flipped equivalent on red. */
  public static Pose3d FlippedPose3d(Pose3d blueValue) {
    return new Pose3d(
        FlippedTranslation3d(blueValue.getTranslation()),
        FlippedRotation3d(blueValue.getRotation()));
  }

  /** Returns the blue rotation, or {@code 180° - blueValue} on red (bilaterally-symmetric mirror). */
  public static Rotation2d MirroredRotation2d(Rotation2d blueValue) {
    return get(blueValue, Rotation2d.k180deg.minus(blueValue));
  }

  /** Returns the blue rotation3d, or the Z-mirrored equivalent on red (bilaterally-symmetric). */
  public static Rotation3d MirroredRotation3d(Rotation3d blueValue) {
    return new Rotation3d(
        blueValue.getX(),
        blueValue.getY(),
        MirroredRotation2d(Rotation2d.fromRadians(blueValue.getZ())).getRadians());
  }

  /** Returns the blue translation, or {@code (fieldLength - x, y)} on red (X-axis mirror only). */
  public static Translation2d MirroredTranslation2d(Translation2d blueValue) {
    return get(
        blueValue, new Translation2d(FIELD_DIMENSIONS.getX() - blueValue.getX(), blueValue.getY()));
  }

  /** Returns the blue pose, or the X-axis-mirrored equivalent on red (bilaterally-symmetric). */
  public static Pose2d MirroredPose2d(Pose2d blueValue) {
    return new Pose2d(
        MirroredTranslation2d(blueValue.getTranslation()),
        MirroredRotation2d(blueValue.getRotation()));
  }

  /** Returns the blue translation3d, or {@code (fieldLength - x, y, z)} on red (X-axis mirror). */
  public static Translation3d MirroredTranslation3d(Translation3d blueValue) {
    return get(
        blueValue,
        new Translation3d(
            FIELD_DIMENSIONS.getX() - blueValue.getX(), blueValue.getY(), blueValue.getZ()));
  }

  /** Returns the blue pose3d, or the X-axis-mirrored equivalent on red. */
  public static Pose3d MirroredPose3d(Pose3d blueValue) {
    return new Pose3d(
        MirroredTranslation3d(blueValue.getTranslation()),
        MirroredRotation3d(blueValue.getRotation()));
  }
}
