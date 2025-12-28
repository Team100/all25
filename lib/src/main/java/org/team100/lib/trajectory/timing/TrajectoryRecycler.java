package org.team100.lib.trajectory.timing;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.PathPoint;
import org.team100.lib.trajectory.Trajectory100;

/**
 * Resamples a trajectory with constant time between samples.
 * 
 * The idea is to prevent overworking the fast parts (where few samples are
 * required) and underworking the slow parts (where more samples are required),
 * as the spatial sampling tends to do. This allows the overall
 * trajectory-making process to be faster, but have higher resolution where it
 * matters.
 * 
 * The sampling goes all the way back to the source spline, to avoid
 * interpolation error, so the trajectory needs to include it.
 * 
 * The new samples are used to reschedule a new trajectory which may have a
 * different duration.
 */
public class TrajectoryRecycler {

    public static Trajectory100 recycle(Trajectory100 original, double dt) {
        double start_vel = original.sample(0).velocityM_S();
        double end_vel = original.getLastPoint().velocityM_S();

        List<PathPoint> points = new ArrayList<>();
        // a little past the end to make sure we get the last point.
        for (double t = 0; t < original.duration() + dt; t += dt) {
            TimedState sample = original.sample(t);
            // these contain discretization errors
            // TODO: go back to the original spline for these
            PathPoint point = sample.point();
            points.add(point);
        }
        return new TrajectoryFactory(original.m_constraints)
                .fromSamples(points.toArray(new PathPoint[0]), start_vel, end_vel);

    }

}
