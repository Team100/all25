package org.team100.lib.trajectory;

import java.util.List;

import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.path.Path100;
import org.team100.lib.trajectory.path.PathFactory;
import org.team100.lib.trajectory.timing.TrajectoryFactory;

/**
 * Creates a trajectory in three steps:
 * 
 * 1. create a spline
 * 2. create points along the spline so that the secants between the points are
 * within the spline sample tolerance, and the points are close enough together
 * 3. assign timestamps to each step
 * 
 * This used to support moving start and end states but we never used it, so
 * it's gone.
 */
public class TrajectoryPlanner {
    private static final boolean DEBUG = false;

    private final PathFactory m_pathFactory;
    private final TrajectoryFactory m_trajectoryFactory;

    public TrajectoryPlanner(PathFactory pathFactory, TrajectoryFactory trajectoryFactory) {
        m_pathFactory = pathFactory;
        m_trajectoryFactory = trajectoryFactory;
    }

    /**
     * Makes a trajectory through the supplied waypoints, starting and ending
     * motionless.
     */
    public Trajectory100 restToRest(List<WaypointSE2> waypoints) {
        return generateTrajectory(waypoints, 0.0, 0.0);
    }

    /////////////////////////////////////////////////////////////////////////////////////
    ///
    /// DANGER ZONE
    /// 
    /// Only use this if you know what you're doing.

    /**
     * Makes a trajectory through the supplied waypoints, with start and end
     * velocities.
     */
    public Trajectory100 generateTrajectory(
            List<WaypointSE2> waypoints, double start_vel, double end_vel) {
        try {
            // Create a path from splines.
            Path100 path = m_pathFactory.fromWaypoints(waypoints);
            if (DEBUG)
                System.out.printf("PATH\n%s\n", path);
            // Generate the timed trajectory.
            Trajectory100 result = m_trajectoryFactory.fromPath(path, start_vel, end_vel);
            if (DEBUG)
                System.out.printf("TRAJECTORY\n%s\n", result);
            return result;
        } catch (IllegalArgumentException e) {
            // catches various kinds of malformed input, returns a no-op.
            // this should never actually happen.
            System.out.println("WARNING: Bad trajectory input!!");
            // print the stack trace if you want to know who is calling
            // e.printStackTrace();
            return new Trajectory100();
        }
    }
}
