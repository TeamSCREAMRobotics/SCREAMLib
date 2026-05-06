package com.teamscreamrobotics.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

import java.util.function.Supplier;

/**
 * Static factory methods for BLine drive-to-pose commands.
 *
 * <p>All methods return a {@code Commands.deferredProxy}-wrapped command so path construction
 * is deferred until schedule time, making them safe to store and re-schedule without
 * capturing a stale robot pose.
 */
public final class BLineUtil {
    /**
     * Drives to a fixed pose; the path is constructed fresh each time the command is scheduled.
     */
    public static Command driveToPose(FollowPath.Builder builder, Pose2d target) {
        return Commands.deferredProxy(
                () -> builder.build(new Path(new Path.Waypoint(target))))
                .withName("BLineDriveToPose(" + target.getX() + ", " + target.getY() + ")");
    }

    /**
     * Drives to a dynamically-supplied pose; the supplier is called at schedule time.
     */
    public static Command driveToPose(FollowPath.Builder builder, Supplier<Pose2d> target) {
        return Commands.deferredProxy(
                () -> builder.build(new Path(new Path.Waypoint(target.get()))))
                .withName("BLineDriveToPose(dynamic)");
    }

    /**
     * Drives to a fixed pose with custom per-path constraints (speed limits, tolerances).
     */
    public static Command driveToPose(FollowPath.Builder builder, Pose2d target,
                                  Path.PathConstraints constraints) {
        return Commands.deferredProxy(
                () -> builder.build(new Path(constraints, new Path.Waypoint(target))))
                .withName("BLineDriveToPose(" + target.getX() + ", " + target.getY() + ")");
    }

    /**
     * Drives to a dynamically-supplied pose with custom constraints; both are evaluated at schedule time.
     */
    public static Command driveToPose(FollowPath.Builder builder, Supplier<Pose2d> target,
                                  Path.PathConstraints constraints) {
        return Commands.deferredProxy(
                () -> builder.build(new Path(constraints, new Path.Waypoint(target.get()))))
                .withName("BLineDriveToPose(dynamic)");
    }

    /**
     * Drives to a fixed translation without commanding any heading change.
     */
    public static Command driveToTranslation(FollowPath.Builder builder, Translation2d target) {
        return Commands.deferredProxy(
                () -> builder.build(new Path(new Path.TranslationTarget(target))))
                .withName("BLineDriveToTranslation(" + target.getX() + ", " + target.getY() + ")");
    }

    /**
     * Drives to a dynamically-supplied translation without commanding any heading change;
     * the supplier is called at schedule time.
     */
    public static Command driveToTranslation(FollowPath.Builder builder, Supplier<Translation2d> target) {
        return Commands.deferredProxy(
                () -> builder.build(new Path(new Path.TranslationTarget(target.get()))))
                .withName("BLineDriveToTranslation(dynamic)");
    }
}
