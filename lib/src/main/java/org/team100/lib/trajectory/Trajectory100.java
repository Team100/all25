package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.timing.TimedPose;

/**
 * Represents a 2d path with heading and a schedule.
 * 
 * As of 2023 a trajectory is not two things (path and heading) it's one thing, each path point includes heading.
 */
public class Trajectory100 {
    protected final List<TimedPose> m_points;

    public Trajectory100() {
        m_points = new ArrayList<>();
    }

    public Trajectory100(final List<TimedPose> states) {
        m_points = states;
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

    public double getTotalTimeSeconds() {
        return getLastPoint().getTimeS();
    }

    public TimedPose getPoint(final int index) {
        return m_points.get(index);
    }

    public List<TimedPose> getPoints() {
        return m_points;
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < length(); ++i) {
            builder.append(i);
            builder.append(": state: ");
            builder.append(getPoint(i).state());
            builder.append(System.lineSeparator());
        }
        return builder.toString();
    }
}
