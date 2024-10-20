package util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.BooleanSupplier;

public final class AllianceFlipUtil {

  public static final Translation2d FIELD_DIMENSIONS = new Translation2d(16.541, 8.211);
  public static final double FIELD_WIDTH = FIELD_DIMENSIONS.getY();
  public static final double FIELD_LENGTH = FIELD_DIMENSIONS.getX();

  public static BooleanSupplier shouldFlip() {
    return () -> DriverStation.getAlliance().filter(value -> value == Alliance.Red).isPresent();
  }

  public static Rotation2d getFwdHeading() {
    return get(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180));
  }

  public static int getDirectionCoefficient() {
    return (int) get(1, -1);
  }

  public static <T> T get(T blueValue, T redValue) {
    return get(blueValue, redValue, false);
  }

  public static <T> T get(T blueValue, T redValue, boolean inverse) {
    return (inverse && !shouldFlip().getAsBoolean() || !inverse && shouldFlip().getAsBoolean())
        ? redValue
        : blueValue;
  }

  public static Rotation2d MirroredRotation2d(Rotation2d blueValue) {
    return get(blueValue, new Rotation2d(Math.PI).minus(blueValue));
  }

  public static Rotation3d MirroredRotation3d(Rotation3d blueValue) {
    return new Rotation3d(
        blueValue.getX(),
        blueValue.getY(),
        MirroredRotation2d(Rotation2d.fromRadians(blueValue.getZ())).getRadians());
  }

  public static Translation2d MirroredTranslation2d(Translation2d blueValue) {
    return get(
        blueValue, new Translation2d(FIELD_DIMENSIONS.getX() - blueValue.getX(), blueValue.getY()));
  }

  public static Pose2d MirroredPose2d(Pose2d blueValue) {
    return new Pose2d(
        MirroredTranslation2d(blueValue.getTranslation()),
        MirroredRotation2d(blueValue.getRotation()));
  }

  public static Translation3d MirroredTranslation3d(Translation3d blueValue) {
    return get(
        blueValue,
        new Translation3d(
            FIELD_DIMENSIONS.getX() - blueValue.getX(), blueValue.getY(), blueValue.getZ()));
  }

  public static Pose3d MirroredPose3d(Pose3d blueValue) {
    return new Pose3d(
        MirroredTranslation3d(blueValue.getTranslation()),
        MirroredRotation3d(blueValue.getRotation()));
  }
}
