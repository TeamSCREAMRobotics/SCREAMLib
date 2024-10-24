package drivers;

import math.ScreamMath;
import pid.ScreamPIDConstants;
import util.AllianceFlipUtil;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

public class PhoenixSwerveHelper {

  private final PhoenixPIDController snapController;
  private final PIDController headingCorrectionController;

  // private final FieldCentricFacingAngle fieldCentricFacingAngle;
  private final FieldCentric fieldCentric;
  private final RobotCentric robotCentric;
  private final ApplyChassisSpeeds applyChassisSpeeds;

  private final Supplier<Pose2d> poseSup;
  private final double MAX_SPEED;

  private Debouncer correctionDebouncer = new Debouncer(0.2, DebounceType.kRising);
  private Rotation2d lastAngle = AllianceFlipUtil.getFwdHeading();

  public PhoenixSwerveHelper(
      Supplier<Pose2d> poseSup,
      double maxSpeed,
      ScreamPIDConstants snapConstants,
      ScreamPIDConstants headingCorrectionConstants) {
    /*     fieldCentricFacingAngle =
    new FieldCentricFacingAngle()
        .withDeadband(maxSpeed * 0.05)
        .withDriveRequestType(com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType.MotionMagic); */
    fieldCentric =
        new FieldCentric()
            .withDeadband(maxSpeed * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    robotCentric =
        new RobotCentric()
            .withDeadband(maxSpeed * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    applyChassisSpeeds =
        new ApplyChassisSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    this.snapController = snapConstants.getPhoenixPIDController(true);
    /* fieldCentricFacingAngle.HeadingController = snapController;
    fieldCentricFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI); */

    this.headingCorrectionController = headingCorrectionConstants.getPIDController(true);

    this.poseSup = poseSup;
    this.MAX_SPEED = maxSpeed;
  }

  public void setLastAngle(Rotation2d angle) {
    lastAngle = angle;
  }

  public FieldCentric getFacingAngle(Translation2d translation, Rotation2d targetAngle) {
    /* return fieldCentricFacingAngle
    .withVelocityX(translation.getX())
    .withVelocityY(translation.getY())
    .withTargetDirection(targetAngle); */
    return getFieldCentric(
        translation,
        snapController.calculate(
            poseSup.get().getRotation().getRadians(),
            targetAngle.getRadians(),
            Timer.getFPGATimestamp()));
  }

  public FieldCentric getFacingAngle(
      Translation2d translation, Rotation2d targetAngle, ScreamPIDConstants headingConstants) {
    return getFieldCentric(
        translation,
        headingConstants
            .getPIDController()
            .calculate(poseSup.get().getRotation().getRadians(), targetAngle.getRadians()));
  }

  public FieldCentric getFacingAngleProfiled(
      Translation2d translation, Rotation2d targetAngle, ProfiledPIDController profile) {
    return getFieldCentric(
        translation,
        profile.calculate(poseSup.get().getRotation().getRadians(), targetAngle.getRadians()));
  }

  public FieldCentric getFacingAngleCOR(
      Translation2d translation, Rotation2d targetAngle, Translation2d centerOfRotation) {
    return getFacingAngle(translation, targetAngle);
  }

  public FieldCentric getFacingAngleProfiledCOR(
      Translation2d translation,
      Rotation2d targetAngle,
      ProfiledPIDController profile,
      Translation2d centerOfRotation) {
    return getFieldCentricCOR(
        translation,
        profile.calculate(poseSup.get().getRotation().getRadians(), targetAngle.getRadians()),
        centerOfRotation);
  }

  public FieldCentric getPointingAt(
      Translation2d translation, Translation2d targetPoint, boolean facingBackwards) {
    return getFacingAngle(
        translation,
        facingBackwards
            ? ScreamMath.calculateAngleToPoint(poseSup.get().getTranslation(), targetPoint)
                .minus(new Rotation2d(Math.PI))
            : ScreamMath.calculateAngleToPoint(poseSup.get().getTranslation(), targetPoint));
  }

  public FieldCentric getHeadingCorrectedFieldCentric(
      Translation2d translation, double angularVelocity) {
    double omega;
    if (correctionDebouncer.calculate(Math.abs(angularVelocity) < 0.05)) {
      omega =
          headingCorrectionController.calculate(
              poseSup.get().getRotation().getRadians(), lastAngle.getRadians());
    } else {
      omega = angularVelocity;
      lastAngle = poseSup.get().getRotation();
    }
    return fieldCentric
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(omega)
        .withCenterOfRotation(Translation2d.kZero);
  }

  public FieldCentric getFieldCentric(Translation2d translation, double angularVelocity) {
    return fieldCentric
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(angularVelocity)
        .withCenterOfRotation(Translation2d.kZero);
  }

  public FieldCentric getFieldCentricCOR(
      Translation2d translation, double angularVelocity, Translation2d centerOfRotation) {
    return fieldCentric
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(angularVelocity)
        .withCenterOfRotation(Translation2d.kZero);
  }

  public RobotCentric getRobotCentric(Translation2d translation, double angularVelocity) {
    return robotCentric
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(angularVelocity)
        .withCenterOfRotation(Translation2d.kZero);
  }

  public RobotCentric getRobotCentricCOR(
      Translation2d translation, double angularVelocity, Translation2d centerOfRotation) {
    return robotCentric
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(angularVelocity)
        .withCenterOfRotation(centerOfRotation);
  }

  public ApplyChassisSpeeds getApplyChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    return applyChassisSpeeds.withSpeeds(chassisSpeeds);
  }

  public ApplyChassisSpeeds getApplyChassisSpeedsCOR(
      ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
    return applyChassisSpeeds.withSpeeds(chassisSpeeds).withCenterOfRotation(centerOfRotation);
  }
}
