package org.team100.lib.path;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.timing.ScheduleGenerator;

/**
 * Represents a 2d holonomic path, i.e. with heading independent from course.
 * 
 * There's no timing information here. For that, see Trajectory100.
 */
public class Path100 {
    private final List<Pose2dWithMotion> m_points;
    /** in meters */
    private final double[] m_distances;

    public Path100(final List<Pose2dWithMotion> states) {
        m_points = new ArrayList<>(states.size());
        m_distances = new double[states.size()];
        if (states.isEmpty()) {
            return;
        }
        m_distances[0] = 0.0;
        m_points.add(states.get(0));
        for (int i = 1; i < states.size(); ++i) {
            m_points.add(states.get(i));
            m_distances[i] = m_distances[i - 1]
                    + getPoint(i - 1).distance(getPoint(i));
        }
    }

    public boolean isEmpty() {
        return m_points.isEmpty();
    }

    public int length() {
        return m_points.size();
    }

    public Pose2dWithMotion getPoint(final int index) {
        if (m_points.isEmpty())
            return null;
        return m_points.get(index);
    }

    /** This is always non-negative. */
    public double getMaxDistance() {
        if (m_points.isEmpty())
            return 0.0;
        return m_distances[m_distances.length - 1];
    }

    public double getMinDistance() {
        return 0.0;
    }

    /**
     * @param distance in meters, always a non-negative number.
     */
    public Pose2dWithMotion sample(double distance) throws ScheduleGenerator.TimingException {
        if (distance >= getMaxDistance()) {
            Pose2dWithMotion point = getPoint(length() - 1);
            return point;
        }
        if (distance <= 0.0) {
            Pose2dWithMotion point = getPoint(0);
            return point;
        }
        for (int i = 1; i < m_distances.length; ++i) {
            final Pose2dWithMotion point = getPoint(i);
            if (m_distances[i] >= distance) {
                final Pose2dWithMotion prev_s = getPoint(i - 1);
                if (Math.abs(m_distances[i] - m_distances[i - 1]) <= 1e-12) {
                    return point;
                } else {
                    return prev_s.interpolate(
                            point,
                            (distance - m_distances[i - 1]) / (m_distances[i] - m_distances[i - 1]));
                }
            }
        }
        throw new ScheduleGenerator.TimingException();
    }

    public Pose2dWithMotion getInterpolated(final double index) {
        if (isEmpty()) {
            return null;
        } else if (index <= 0.0) {
            Pose2dWithMotion point = getPoint(0);
            return point;
        } else if (index >= length() - 1) {
            Pose2dWithMotion point = getPoint(length() - 1);
            return point;
        }
        final int i = (int) Math.floor(index);
        final double frac = index - i;
        if (frac <= Double.MIN_VALUE) {
            Pose2dWithMotion point = getPoint(i);
            return point;
        } else if (frac >= 1.0 - Double.MIN_VALUE) {
            Pose2dWithMotion point = getPoint(i + 1);
            return point;
        } else {
            return getPoint(i).interpolate(getPoint(i + 1), frac);
        }
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
