package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.PathPoint;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.timing.TimedState;
import org.team100.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A list of timed poses.
 */
public class Trajectory100 {
    private final List<TimedState> m_points;
    /** Constraints used for this trajectory, for resampling */
    public final List<TimingConstraint> m_constraints;
    private final double m_duration;

    public Trajectory100() {
        m_points = new ArrayList<>();
        m_constraints = new ArrayList<>();
        m_duration = 0;
    }

    /** First timestamp must be zero. */
    public Trajectory100(List<TimedState> states, List<TimingConstraint> constraints) {
        m_points = states;
        m_constraints = constraints;
        m_duration = m_points.get(m_points.size() - 1).getTimeS();
    }

    /**
     * Interpolate a TimedState.
     * 
     * @param timeS start is zero.
     */
    public TimedState sample(double timeS) {
        // This scans the whole trajectory for every sample, but most of the time
        // is the interpolation; I tried a TreeMap index and it only saved a few
        // nanoseconds per call.
        if (isEmpty())
            throw new IllegalStateException("can't sample an empty trajectory");
        if (timeS >= m_duration) {
            return getLastPoint();
        }
        if (timeS <= 0) {
            return getPoint(0);
        }

        for (int i = 1; i < length(); ++i) {
            final TimedState ceil = getPoint(i);
            if (ceil.getTimeS() >= timeS) {
                final TimedState floor = getPoint(i - 1);
                double span = ceil.getTimeS() - floor.getTimeS();
                if (Math.abs(span) <= 1e-12) {
                    return ceil;
                }
                double delta_t = timeS - floor.getTimeS();
                return floor.interpolate(ceil, delta_t);
            }
        }
        throw new IllegalStateException("impossible trajectory: " + toString());
    }

    /** Time is at or beyond the trajectory duration. */
    public boolean isDone(double timeS) {
        return timeS >= duration();
    }

    public boolean isEmpty() {
        return m_points.isEmpty();
    }

    public int length() {
        return m_points.size();
    }

    public TimedState getLastPoint() {
        return m_points.get(length() - 1);
    }

    public double duration() {
        return m_duration;
    }

    public List<TimedState> getPoints() {
        return m_points;
    }

    public TimedState getPoint(int index) {
        return m_points.get(index);
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < length(); ++i) {
            builder.append(i);
            builder.append(": ");
            builder.append(getPoint(i));
            builder.append(System.lineSeparator());
        }
        return builder.toString();
    }

    /** For cutting-and-pasting into a spreadsheet */
    public void dump() {
        System.out.println("i, t, v, a, k, x, y");
        for (int i = 0; i < length(); ++i) {
            TimedState ts = getPoint(i);
            PathPoint pwm = ts.point();
            WaypointSE2 w = pwm.waypoint();
            Pose2d p = w.pose();
            System.out.printf("%d, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
                    i, ts.getTimeS(), ts.velocityM_S(), ts.acceleration(), pwm.getCurvatureRad_M(), p.getX(), p.getY());
        }
    }

    /** For cutting-and-pasting into a spreadsheet */
    public void tdump() {
        System.out.println("t, v, a, k, x, y");
        for (double t = 0; t < duration(); t += 0.02) {
            TimedState ts = sample(t);
            PathPoint pwm = ts.point();
            WaypointSE2 w = pwm.waypoint();
            Pose2d p = w.pose();
            System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
                    ts.getTimeS(), ts.velocityM_S(), ts.acceleration(), pwm.getCurvatureRad_M(), p.getX(), p.getY());
        }
    }
}
