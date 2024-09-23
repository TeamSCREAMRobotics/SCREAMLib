package com.SCREAMLib.drivers;

import com.SCREAMLib.math.ScreamMath;
import com.SCREAMLib.pid.ScreamPIDConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

public class PhoenixSwerveHelper {

  private final PhoenixPIDController headingController;

  private final FieldCentricFacingAngle fieldCentricFacingAngle;
  private final FieldCentric fieldCentric;
  private final RobotCentric robotCentric;
  private final ApplyChassisSpeeds applyChassisSpeeds;

  private final Supplier<Pose2d> poseSup;
  private final double MAX_SPEED;
  private final double MAX_ANGULAR_SPEED;

  public PhoenixSwerveHelper(
      Supplier<Pose2d> poseSup,
      double maxAngularSpeed,
      double maxSpeed,
      ScreamPIDConstants snapConstants) {
    fieldCentricFacingAngle =
        new FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.05)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagic);
    fieldCentric =
        new FieldCentric()
            .withDeadband(maxSpeed * 0.05)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagic);
    robotCentric =
        new RobotCentric()
            .withDeadband(maxSpeed * 0.05)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagic);
    applyChassisSpeeds =
        new ApplyChassisSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagic);

    this.headingController = snapConstants.getPhoenixPIDController();
    fieldCentricFacingAngle.HeadingController = headingController;
    fieldCentricFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    this.poseSup = poseSup;
    this.MAX_ANGULAR_SPEED = maxAngularSpeed;
    this.MAX_SPEED = maxSpeed;
  }

  public SwerveRequest getFacingAngle(Translation2d translation, Rotation2d targetAngle) {
    Translation2d xy = translation.times(MAX_SPEED);
    return fieldCentricFacingAngle
        .withVelocityX(xy.getX())
        .withVelocityY(xy.getY())
        .withTargetDirection(targetAngle);
  }

  public SwerveRequest getFacingAngleProfiled(
      Translation2d translation, Rotation2d targetAngle, ProfiledPIDController profile) {
    return getFieldCentric(
        translation,
        profile.calculate(poseSup.get().getRotation().getRadians(), targetAngle.getRadians()));
  }

  public SwerveRequest getFacingAngleCOR(
      Translation2d translation, Rotation2d targetAngle, Translation2d centerOfRotation) {
    Translation2d xy = translation.times(MAX_SPEED);
    return fieldCentricFacingAngle
        .withVelocityX(xy.getX())
        .withVelocityY(xy.getY())
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

  public SwerveRequest getFieldCentric(Translation2d translation, double angularVelocity) {
    Translation2d xy = translation.times(MAX_SPEED);
    double omega = angularVelocity * MAX_ANGULAR_SPEED;
    return fieldCentric
        .withVelocityX(xy.getX())
        .withVelocityY(xy.getY())
        .withRotationalRate(omega)
        .withCenterOfRotation(new Translation2d());
  }

  public SwerveRequest getFieldCentricCOR(
      Translation2d translation, double angularVelocity, Translation2d centerOfRotation) {
    Translation2d xy = translation.times(MAX_SPEED);
    double omega = angularVelocity * MAX_ANGULAR_SPEED;
    return fieldCentric
        .withVelocityX(xy.getX())
        .withVelocityY(xy.getY())
        .withRotationalRate(omega)
        .withCenterOfRotation(centerOfRotation);
  }

  public SwerveRequest getRobotCentric(Translation2d translation, double angularVelocity) {
    Translation2d xy = translation.times(MAX_SPEED);
    double omega = angularVelocity * MAX_ANGULAR_SPEED;
    return robotCentric.withVelocityX(xy.getX()).withVelocityY(xy.getY()).withRotationalRate(omega);
  }

  public SwerveRequest getRobotCentricCOR(
      Translation2d translation, double angularVelocity, Translation2d centerOfRotation) {
    Translation2d xy = translation.times(MAX_SPEED);
    double omega = angularVelocity * MAX_ANGULAR_SPEED;
    return robotCentric
        .withVelocityX(xy.getX())
        .withVelocityY(xy.getY())
        .withRotationalRate(omega)
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
