package org.team100.lib.geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * For constructing splines, paths, and trajectories.
 * 
 * @param translation
 * @param heading
 * @param course
 */
public record HolonomicPose2d(
        Translation2d translation,
        Rotation2d heading,
        Rotation2d course) {
}
