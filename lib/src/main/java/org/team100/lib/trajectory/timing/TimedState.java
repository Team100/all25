package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.PathPoint;
import org.team100.lib.util.Math100;

/**
 * Represents a state within a 2d holonomic trajectory, i.e. with heading
 * independent from course.
 * 
 * The timing fields are set by the TrajectoryFactory.
 */
public class TimedState {
    private static final boolean DEBUG = false;
    private final PathPoint m_state;
    /** Time we achieve this state. */
    private final double m_timeS;
    /** Instantaneous pathwise velocity, m/s. */
    private final double m_velocityM_S;
    /**
     * Pathwise acceleration for the timespan after this state, m/s^2. It's computed
     * by looking at the velocity of the next state, and the distance to get there.
     */
    private final double m_accelM_S_S;

    public TimedState(
            PathPoint state,
            double t,
            double velocity,
            double acceleration) {
        m_state = state;
        m_timeS = t;
        m_velocityM_S = velocity;
        m_accelM_S_S = acceleration;
    }

    public PathPoint state() {
        return m_state;
    }

    /** Time we achieve this state. */
    public double getTimeS() {
        return m_timeS;
    }

    /** Instantaneous pathwise velocity, m/s. */
    public double velocityM_S() {
        return m_velocityM_S;
    }

    /** Instantaneous pathwise (not centripetal) acceleration, m/s^2. */
    public double acceleration() {
        return m_accelM_S_S;
    }

    @Override
    public String toString() {
        return String.format("state %s, time %5.3f, vel %5.3f, acc %5.3f",
                m_state,
                m_timeS,
                m_velocityM_S,
                m_accelM_S_S);
    }

    /**
     * Linear interpolation by time.
     * 
     * Velocity of this state is the initial velocity.
     * Acceleration of this state is constant through the whole arc.
     */
    public TimedState interpolate(TimedState other, double delta_t) {
        double tLerp = m_timeS + delta_t;
        double vLerp = m_velocityM_S + m_accelM_S_S * delta_t;
        double pathwiseDistance = m_velocityM_S * delta_t + 0.5 * m_accelM_S_S * delta_t * delta_t;

        double distanceBetween = m_state.distanceCartesian(other.m_state);
        double interpolant = pathwiseDistance / distanceBetween;
        if (Double.isNaN(interpolant)) {
            interpolant = 1.0;
        }

        if (DEBUG)
            System.out.printf("tlerp %f\n", tLerp);
        return new TimedState(
                m_state.interpolate(other.m_state, interpolant),
                tLerp,
                vLerp,
                m_accelM_S_S);
    }

    /** Translation only, ignores rotation */
    public double distanceCartesian(TimedState other) {
        return m_state.distanceCartesian(other.m_state);
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof TimedState)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }
        TimedState ts = (TimedState) other;
        if (!m_state.equals(ts.m_state)) {
            if (DEBUG)
                System.out.println("wrong state");
            return false;
        }
        if (!Math100.epsilonEquals(m_timeS, ts.m_timeS)) {
            if (DEBUG)
                System.out.println("wrong time");
            return false;
        }
        return true;
    }
}
