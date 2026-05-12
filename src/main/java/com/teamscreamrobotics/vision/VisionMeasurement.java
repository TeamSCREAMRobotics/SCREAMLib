package com.teamscreamrobotics.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * An accepted vision measurement ready to be passed to a pose estimator.
 * The {@code stdDevs} matrix is {@code VecBuilder.fill(xy, xy, theta)}.
 */
public record VisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {}
