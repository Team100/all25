package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.timing.TimedPose;

/**
 * A list of timed poses.
 */
public class Trajectory100 {
    private final List<TimedPose> m_points;
    private final double m_duration;

    public Trajectory100() {
        m_points = new ArrayList<>();
        m_duration = 0;
    }

    /** First timestamp must be zero. */
    public Trajectory100(final List<TimedPose> states) {
        m_points = states;
        m_duration = m_points.get(m_points.size() - 1).getTimeS();
    }

    /**
     * Interpolate a TimedPose.
     * 
     * This scans the whole trajectory for every sample, but most of the time
     * is the interpolation; I tried a TreeMap index and it only saved a few
     * nanoseconds per call.
     * 
     * @param timeS start is zero.
     */
    public TimedPose sample(final double timeS) {
        if (isEmpty())
            throw new IllegalStateException("can't sample an empty trajectory");
        if (timeS >= m_duration) {
            return getLastPoint();
        }
        if (timeS <= 0) {
            return getPoint(0);
        }

        for (int i = 1; i < length(); ++i) {
            final TimedPose ceil = getPoint(i);
            if (ceil.getTimeS() >= timeS) {
                final TimedPose floor = getPoint(i - 1);
                double betweenPoints = ceil.getTimeS() - floor.getTimeS();
                if (Math.abs(betweenPoints) <= 1e-12) {
                    return ceil;
                }
                double t = (timeS - floor.getTimeS()) / betweenPoints;
                return floor.interpolate2(ceil, t);
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

    public TimedPose getLastPoint() {
        return m_points.get(length() - 1);
    }

    public double duration() {
        return m_duration;
    }

    public List<TimedPose> getPoints() {
        return m_points;
    }

    TimedPose getPoint(int index) {
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
}
