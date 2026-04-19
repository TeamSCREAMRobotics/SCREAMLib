package com.teamscreamrobotics.drivers;

import com.teamscreamrobotics.math.ScreamMath;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
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
import java.util.function.Supplier;

/**
 * Factory for CTRE Phoenix 6 swerve drive requests with built-in heading snap and correction.
 * Wraps {@link FieldCentric}, {@link FieldCentricFacingAngle}, {@link RobotCentric}, and
 * {@link ApplyRobotSpeeds}/{@link ApplyFieldSpeeds} requests with pre-configured deadbands and
 * drive/steer request types.
 */
public class PhoenixSwerveHelper {

  private final PhoenixPIDController snapController;
  private final PIDController headingCorrectionController;

  private final FieldCentricFacingAngle fieldCentricFacingAngle;
  private final FieldCentric fieldCentric;
  private final RobotCentric robotCentric;
  private final ApplyRobotSpeeds applyRobotSpeeds;

  private final ApplyFieldSpeeds applyFieldSpeeds;

  private final Supplier<Pose2d> poseSup;

  private Debouncer correctionDebouncer = new Debouncer(0.2, DebounceType.kRising);
  private Rotation2d lastAngle = AllianceFlipUtil.getFwdHeading();

  /**
   * Creates a new helper with configured swerve requests.
   *
   * @param poseSup                    supplier of the robot's current field pose
   * @param maxSpeed                   drivetrain maximum speed in m/s (used to set deadbands)
   * @param snapConstants              PID constants for the heading snap controller
   * @param headingCorrectionConstants PID constants for the heading drift correction controller
   */
  public PhoenixSwerveHelper(
      Supplier<Pose2d> poseSup,
      double maxSpeed,
      ScreamPIDConstants snapConstants,
      ScreamPIDConstants headingCorrectionConstants) {
    fieldCentricFacingAngle =
    new FieldCentricFacingAngle()
        .withDeadband(maxSpeed * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
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
    applyRobotSpeeds =
        new ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    applyFieldSpeeds =
        new ApplyFieldSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    this.snapController = snapConstants.getPhoenixPIDController(-Math.PI, Math.PI);
    fieldCentricFacingAngle.HeadingController = snapController;

    this.headingCorrectionController = headingCorrectionConstants.getPIDController(-Math.PI, Math.PI);

    this.poseSup = poseSup;
  }

  /** Overrides the heading correction hold angle — call this when intentionally changing heading. */
  public void setLastAngle(Rotation2d angle) {
    lastAngle = angle;
  }

  /**
   * Returns a {@link FieldCentricFacingAngle} request using the built-in snap controller.
   *
   * @param translation field-relative XY velocity vector
   * @param targetAngle desired heading
   */
  public FieldCentricFacingAngle getFacingAngle(Translation2d translation, Rotation2d targetAngle) {
    return fieldCentricFacingAngle
    .withVelocityX(translation.getX())
    .withVelocityY(translation.getY())
    .withTargetDirection(targetAngle);
  }

  /**
   * Returns a {@link FieldCentric} request with angular velocity computed from a custom PID controller.
   *
   * @param translation     field-relative XY velocity vector
   * @param targetAngle     desired heading
   * @param headingController PID constants for heading control (uses a fresh controller per call)
   */
  public FieldCentric getFacingAngle(
      Translation2d translation, Rotation2d targetAngle, PIDController headingController) {
    return getFieldCentric(
        translation,
        headingController
            .calculate(poseSup.get().getRotation().getRadians(), targetAngle.getRadians()));
  }

  /**
   * Returns a {@link FieldCentric} request with angular velocity from a profiled PID controller.
   *
   * @param translation field-relative XY velocity vector
   * @param targetAngle desired heading
   * @param profile     profiled PID controller for smooth heading control
   */
  public FieldCentric getFacingAngleProfiled(
      Translation2d translation, Rotation2d targetAngle, ProfiledPIDController profile) {
    return getFieldCentric(
        translation,
        profile.calculate(poseSup.get().getRotation().getRadians(), targetAngle.getRadians()));
  }

  /**
   * Returns a {@link FieldCentricFacingAngle} request using the built-in snap controller and a custom center of rotation.
   *
   * @param translation      field-relative XY velocity vector
   * @param targetAngle      desired heading
   * @param centerOfRotation offset from robot center for rotation
   */
  public FieldCentricFacingAngle getFacingAngleCOR(
      Translation2d translation, Rotation2d targetAngle, Translation2d centerOfRotation) {
    return getFacingAngle(translation, targetAngle).withCenterOfRotation(centerOfRotation);
  }

  /**
   * Returns a {@link FieldCentric} request with profiled heading and a custom center of rotation.
   *
   * @param translation      field-relative XY velocity vector
   * @param targetAngle      desired heading
   * @param profile          profiled PID controller
   * @param centerOfRotation offset from robot center for rotation
   */
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

  /**
   * Returns a {@link FieldCentricFacingAngle} request that keeps the robot aimed at a field point.
   *
   * @param translation field-relative XY velocity vector
   * @param targetPoint the field point to face
   */
  public FieldCentricFacingAngle getPointingAt(
      Translation2d translation, Translation2d targetPoint) {
    return getPointingAt(translation, targetPoint, Rotation2d.kZero);
  }

  /**
   * Returns a {@link FieldCentricFacingAngle} request that keeps the robot aimed at a field point
   * with an angular offset added to the computed heading.
   *
   * @param translation field-relative XY velocity vector
   * @param targetPoint the field point to face
   * @param offset      additional rotation added to the target heading
   */
  public FieldCentricFacingAngle getPointingAt(
      Translation2d translation, Translation2d targetPoint, Rotation2d offset) {
    return getFacingAngle(
        translation,
        ScreamMath.calculateAngleToPoint(poseSup.get().getTranslation(), targetPoint)
                .plus(offset));
  }

  /**
   * Returns a {@link FieldCentric} request that aims at a field point using a profiled controller.
   *
   * @param translation field-relative XY velocity vector
   * @param targetPoint the field point to face
   * @param profile     profiled PID controller for smooth heading tracking
   */
  public FieldCentric getPointingAtProfiled(
      Translation2d translation, Translation2d targetPoint, ProfiledPIDController profile) {
    return getPointingAtProfiled(translation, targetPoint, new Rotation2d(), profile);
  }

  /**
   * Returns a {@link FieldCentric} request that aims at a field point with an offset, using a
   * profiled controller.
   *
   * @param translation field-relative XY velocity vector
   * @param targetPoint the field point to face
   * @param offset      additional rotation added to the target heading
   * @param profile     profiled PID controller
   */
  public FieldCentric getPointingAtProfiled(
      Translation2d translation, Translation2d targetPoint, Rotation2d offset, ProfiledPIDController profile) {
    return getFacingAngleProfiled(translation, ScreamMath.calculateAngleToPoint(poseSup.get().getTranslation(), targetPoint).plus(offset), profile);
  }

  /**
   * Returns a field-centric request with automatic heading drift correction.
   * While {@code angularVelocity} is near zero the correction controller holds the last known
   * heading; otherwise the driver input is passed through and the hold angle is updated.
   *
   * @param translation     field-relative XY velocity vector
   * @param angularVelocity driver-commanded rotation rate (rad/s)
   */
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

  /**
   * Returns a basic field-centric drive request.
   *
   * @param translation     field-relative XY velocity vector
   * @param angularVelocity rotation rate in rad/s
   */
  public FieldCentric getFieldCentric(Translation2d translation, double angularVelocity) {
    return fieldCentric
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(angularVelocity)
        .withCenterOfRotation(Translation2d.kZero);
  }

  /**
   * Returns a field-centric drive request with a custom center of rotation.
   *
   * @param translation      field-relative XY velocity vector
   * @param angularVelocity  rotation rate in rad/s
   * @param centerOfRotation offset from robot center for rotation
   */
  public FieldCentric getFieldCentricCOR(
      Translation2d translation, double angularVelocity, Translation2d centerOfRotation) {
    return fieldCentric
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(angularVelocity)
        .withCenterOfRotation(Translation2d.kZero);
  }

  /**
   * Returns a robot-centric drive request.
   *
   * @param translation     robot-relative XY velocity vector
   * @param angularVelocity rotation rate in rad/s
   */
  public RobotCentric getRobotCentric(Translation2d translation, double angularVelocity) {
    return robotCentric
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(angularVelocity)
        .withCenterOfRotation(Translation2d.kZero);
  }

  /**
   * Returns a robot-centric drive request with a custom center of rotation.
   *
   * @param translation      robot-relative XY velocity vector
   * @param angularVelocity  rotation rate in rad/s
   * @param centerOfRotation offset from robot center for rotation
   */
  public RobotCentric getRobotCentricCOR(
      Translation2d translation, double angularVelocity, Translation2d centerOfRotation) {
    return robotCentric
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(angularVelocity)
        .withCenterOfRotation(centerOfRotation);
  }

  /** Returns an {@link ApplyRobotSpeeds} request for closed-loop velocity control. */
  public ApplyRobotSpeeds getApplyRobotSpeeds(ChassisSpeeds chassisSpeeds) {
    return applyRobotSpeeds.withSpeeds(chassisSpeeds);
  }

  /**
   * Returns an {@link ApplyRobotSpeeds} request with a custom center of rotation.
   *
   * @param chassisSpeeds    desired robot-relative speeds
   * @param centerOfRotation offset from robot center for rotation
   */
  public ApplyRobotSpeeds getApplyRobotSpeedsCOR(
      ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
    return getApplyRobotSpeeds(chassisSpeeds).withCenterOfRotation(centerOfRotation);
  }

  /** Returns an {@link ApplyFieldSpeeds} request for closed-loop field-relative velocity control. */
  public ApplyFieldSpeeds getApplyFieldSpeeds(ChassisSpeeds chassisSpeeds) {
    return applyFieldSpeeds.withSpeeds(chassisSpeeds);
  }

  /**
   * Returns an {@link ApplyFieldSpeeds} request with a custom center of rotation.
   *
   * @param chassisSpeeds    desired field-relative speeds
   * @param centerOfRotation offset from robot center for rotation
   */
  public ApplyFieldSpeeds getApplyFieldSpeedsCOR(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
    return getApplyFieldSpeeds(chassisSpeeds).withCenterOfRotation(centerOfRotation);
  }
}
