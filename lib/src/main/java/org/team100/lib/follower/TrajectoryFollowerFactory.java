package org.team100.lib.follower;

import org.team100.lib.logging.LoggerFactory;

/**
 * Known-good controller settings.
 */
public class TrajectoryFollowerFactory {

    public static TrajectoryFollower fieldRelativeFancyPIDF(LoggerFactory log) {
        return new TrajectoryFollower(log, new FollowerController(log, 2.4, 1.3, 0.1, 0.1, 0.01, 0.02, 0.01, 0.02));
    }

    public static TrajectoryFollower fieldRelativeGoodPIDF(LoggerFactory log) {
        return new TrajectoryFollower(log, new FollowerController(log, 1, 1.3, 0.1, 0.1, 0.01, 0.02, 0.01, 0.02));
    }

    public static TrajectoryFollower autoFieldRelativePIDF(LoggerFactory log) {
        return new TrajectoryFollower(log, new FollowerController(log, 1, 1.3, 0.1, 0.1, 0.01, 0.02, 0.01, 0.02));
    }

    public static TrajectoryFollower fieldRelativeFfOnly(LoggerFactory log) {
        return new TrajectoryFollower(log, new FollowerController(log, 0, 0, 0, 0, 0.01, 0.02, 0.01, 0.02));
    }

    public static TrajectoryFollower testFieldRelativePIDF(LoggerFactory log) {
        return new TrajectoryFollower(log, new FollowerController(log, 2.4, 2.4, 0.1, 0.1, 0.01, 0.02, 0.01, 0.02));
    }

    public static TrajectoryFollower testFieldRelativeFFOnly(LoggerFactory log) {
        return new TrajectoryFollower(log, new FollowerController(log, 0, 0, 0, 0, 0.01, 0.02, 0.01, 0.02));
    }

    private TrajectoryFollowerFactory() {
        // don't call this
    }

}
