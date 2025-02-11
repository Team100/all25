package org.team100.lib.trajectory;

import edu.wpi.first.math.MathUtil;

/**
 * Allows iterating over the schedule of a trajectory.
 * 
 * I think what we really want is not these "advance" and "preview" kinds of
 * things, but really just direct access to the sampler would be fine.
 * TODO: so get rid of this.
 */
public class TrajectoryTimeIterator {
    private final Trajectory100 m_trajectory;
    private final double m_startS;
    private final double m_endS;

    /** progress along the trajectory in seconds */
    private double m_timeS = 0.0;
    private TrajectorySamplePoint m_current;

    /**
     * Sets the current sample to the first sample.
     * Sets the iterator time to the time of the first sample.
     */
    public TrajectoryTimeIterator(Trajectory100 trajectory) {
        m_trajectory = trajectory;
        m_startS = m_trajectory.getPoint(0).state().getTimeS();
        m_endS = m_trajectory.getPoint(m_trajectory.length() - 1).state().getTimeS();
        m_current = sample(getStartS());
        m_timeS = getStartS();
    }

    public double getStartS() {
        return m_startS;
    }

    public double getEndS() {
        return m_endS;
    }

    /**
     * Returns empty if no sample can be found. This shouldn't happen, but if it
     * does, there's no reasonable default.
     * 
     * @param timeS seconds
     */
    public TrajectorySamplePoint sample(double timeS) {
        if (timeS >= m_endS) {
            TrajectoryPoint point = m_trajectory.getPoint(m_trajectory.length() - 1);
            return new TrajectorySamplePoint(point.state(), point.index(), point.index());
        }
        if (timeS <= m_startS) {
            TrajectoryPoint point = m_trajectory.getPoint(0);
            return new TrajectorySamplePoint(point.state(), point.index(), point.index());
        }
        for (int i = 1; i < m_trajectory.length(); ++i) {
            final TrajectoryPoint point = m_trajectory.getPoint(i);
            if (point.state().getTimeS() >= timeS) {
                final TrajectoryPoint prev_s = m_trajectory.getPoint(i - 1);
                if (Math.abs(point.state().getTimeS() - prev_s.state().getTimeS()) <= 1e-12) {
                    return new TrajectorySamplePoint(point.state(), point.index(), point.index());
                }
                double t = (timeS - prev_s.state().getTimeS())
                        / (point.state().getTimeS() - prev_s.state().getTimeS());
                return new TrajectorySamplePoint(
                        prev_s.state().interpolate2(point.state(), t), i - 1, i);
            }
        }
        throw new IllegalStateException("impossible trajectory: " + m_trajectory);
        // return Optional.empty();
    }

    @Override
    public String toString() {
        return "TrajectoryTimeSampler [trajectory_=" + m_trajectory + ", startTimeS=" + m_startS + ", endTimeS="
                + m_endS + "]";
    }

    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public double getRemainingProgress() {
        return Math.max(0.0, getEndS() - m_timeS);
    }

    public TrajectorySamplePoint getSample() {
        return m_current;
    }

    /**
     * Sample the trajectory and update the iterator state.
     * 
     * @param additional_progress in seconds
     */
    public TrajectorySamplePoint advance(double additional_progress) {
        m_timeS = MathUtil.clamp(m_timeS + additional_progress, getStartS(), getEndS());
        m_current = sample(m_timeS);
        return m_current;
    }

    /**
     * Sample the trajectory without changing the iterator state.
     * 
     * @param additional_progress in seconds
     */
    public TrajectorySamplePoint preview(double additional_progress) {
        return sample(m_timeS + additional_progress);
    }

}
