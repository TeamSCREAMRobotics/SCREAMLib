package com.SCREAMLib.drivers;

import com.SCREAMLib.math.ScreamMath;
import com.SCREAMLib.pid.ScreamPIDConstants;
import com.SCREAMLib.util.AllianceFlipUtil;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

public class PhoenixSwerveHelper {

  private final PhoenixPIDController snapController;
  private final PIDController headingCorrectionController;

  private final FieldCentricFacingAngle fieldCentricFacingAngle;
  private final FieldCentric fieldCentric;
  private final RobotCentric robotCentric;
  private final ApplyChassisSpeeds applyChassisSpeeds;

  private final Supplier<Pose2d> poseSup;
  private final double MAX_SPEED;

  private Debouncer correctionDebouncer = new Debouncer(0.2, DebounceType.kRising);
  private Rotation2d lastAngle = AllianceFlipUtil.getForwardRotation();

  public PhoenixSwerveHelper(
      Supplier<Pose2d> poseSup,
      double maxSpeed,
      ScreamPIDConstants snapConstants,
      ScreamPIDConstants headingCorrectionConstants) {
    fieldCentricFacingAngle =
        new FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagic);
    fieldCentric =
        new FieldCentric()
            .withDeadband(maxSpeed * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagic);
    robotCentric =
        new RobotCentric()
            .withDeadband(maxSpeed * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagic);
    applyChassisSpeeds =
        new ApplyChassisSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagic);

    this.snapController = snapConstants.getPhoenixPIDController();
    fieldCentricFacingAngle.HeadingController = snapController;
    fieldCentricFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    this.headingCorrectionController = headingCorrectionConstants.getPIDController();
    this.headingCorrectionController.enableContinuousInput(-Math.PI, Math.PI);

    this.poseSup = poseSup;
    this.MAX_SPEED = maxSpeed;
  }

  public void setLastAngle(Rotation2d angle) {
    lastAngle = angle;
  }

  public SwerveRequest getFacingAngle(Translation2d translation, Rotation2d targetAngle) {
    return fieldCentricFacingAngle
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withTargetDirection(targetAngle);
  }

  public SwerveRequest getFacingAngle(
      Translation2d translation, Rotation2d targetAngle, ScreamPIDConstants headingConstants) {
    return getFieldCentric(
        translation,
        headingConstants
            .getPIDController()
            .calculate(poseSup.get().getRotation().getRadians(), targetAngle.getRadians()));
  }

  public SwerveRequest getFacingAngleProfiled(
      Translation2d translation, Rotation2d targetAngle, ProfiledPIDController profile) {
    return getFieldCentric(
        translation,
        profile.calculate(poseSup.get().getRotation().getRadians(), targetAngle.getRadians()));
  }

  public SwerveRequest getFacingAngleCOR(
      Translation2d translation, Rotation2d targetAngle, Translation2d centerOfRotation) {
    return fieldCentricFacingAngle
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withTargetDirection(targetAngle)
        .withCenterOfRotation(centerOfRotation);
  }

  public SwerveRequest getFacingAngleProfiledCOR(
      Translation2d translation,
      Rotation2d targetAngle,
      ProfiledPIDController profile,
      Translation2d centerOfRotation) {
    return getFieldCentricCOR(
        translation,
        profile.calculate(poseSup.get().getRotation().getRadians(), targetAngle.getRadians()),
        centerOfRotation);
  }

  public SwerveRequest getPointingAt(Translation2d translation, Translation2d targetPoint) {
    return getFacingAngle(
        translation, ScreamMath.calculateAngleToPoint(poseSup.get().getTranslation(), targetPoint));
  }

  public SwerveRequest getPointingAt(
      Translation2d translation, Translation2d targetPoint, boolean facingBackwards) {
    return getFacingAngle(
        translation,
        facingBackwards
            ? ScreamMath.calculateAngleToPoint(poseSup.get().getTranslation(), targetPoint)
                .minus(new Rotation2d(Math.PI))
            : ScreamMath.calculateAngleToPoint(poseSup.get().getTranslation(), targetPoint));
  }

  public SwerveRequest getHeadingCorrectedFieldCentric(
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

  public SwerveRequest getFieldCentric(Translation2d translation, double angularVelocity) {
    return fieldCentric
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(angularVelocity)
        .withCenterOfRotation(Translation2d.kZero);
  }

  public SwerveRequest getFieldCentricCOR(
      Translation2d translation, double angularVelocity, Translation2d centerOfRotation) {
    return fieldCentric
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(angularVelocity)
        .withCenterOfRotation(Translation2d.kZero);
  }

  public SwerveRequest getRobotCentric(Translation2d translation, double angularVelocity) {
    return robotCentric
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(angularVelocity)
        .withCenterOfRotation(Translation2d.kZero);
  }

  public SwerveRequest getRobotCentricCOR(
      Translation2d translation, double angularVelocity, Translation2d centerOfRotation) {
    return robotCentric
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(angularVelocity)
        .withCenterOfRotation(centerOfRotation);
  }

  public SwerveRequest getApplyChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    return applyChassisSpeeds.withSpeeds(chassisSpeeds);
  }

  public SwerveRequest getApplyChassisSpeedsCOR(
      ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
    return applyChassisSpeeds.withSpeeds(chassisSpeeds).withCenterOfRotation(centerOfRotation);
  }
}
