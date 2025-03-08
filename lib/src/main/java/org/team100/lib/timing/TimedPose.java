package org.team100.lib.timing;

import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.MathUtil;

/**
 * Represents a state within a 2d holonomic trajectory, i.e. with heading
 * independent from course.
 * 
 * The timing fields are set by the ScheduleGenerator.
 */
public class TimedPose {
    private final Pose2dWithMotion m_state;
    private final double m_timeS; // Time we achieve this state.
    private final double m_velocityM_S; // ds/dt
    private double m_accelM_S_S; // d^2s/dt^2

    public TimedPose(
            Pose2dWithMotion state,
            double t,
            double velocity,
            double acceleration) {
        m_state = state;
        m_timeS = t;
        m_velocityM_S = velocity;
        m_accelM_S_S = acceleration;
    }

    public Pose2dWithMotion state() {
        return m_state;
    }

    public double getTimeS() {
        return m_timeS;
    }

    public double velocityM_S() {
        return m_velocityM_S;
    }

    /** accel is set based on the velocity of the next state, so we set it here. */
    void set_acceleration(double acceleration) {
        m_accelM_S_S = acceleration;
    }

    /** this means acceleration along the path, not centripetal acceleration. */
    public double acceleration() {
        return m_accelM_S_S;
    }

    @Override
    public String toString() {
        return String.format("state %s, time %5.3f, vel %5.3f, acc %5.3f",
                state(),
                getTimeS(),
                velocityM_S(),
                acceleration());
    }

    public TimedPose interpolate2(TimedPose other, double x) {
        final double new_t = MathUtil.interpolate(getTimeS(), other.getTimeS(), x);
        final double delta_t = new_t - getTimeS();
        if (delta_t < 0.0) {
            return other.interpolate2(this, 1.0 - x);
        }
        boolean reversing = velocityM_S() < 0.0 || (Math.abs(velocityM_S() - 0.0) <= 1e-12 && acceleration() < 0.0);
        final double new_v = velocityM_S() + acceleration() * delta_t;
        final double new_s = (reversing ? -1.0 : 1.0)
                * (velocityM_S() * delta_t + .5 * acceleration() * delta_t * delta_t);

        double interpolant = new_s / state().distance(other.state());
        if (Double.isNaN(interpolant)) {
            interpolant = 1.0;
        }

        return new TimedPose(
                state().interpolate(other.state(), interpolant),
                new_t,
                new_v,
                acceleration());
    }

    public double distance(TimedPose other) {
        return state().distance(other.state());
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof TimedPose)) {
            return false;
        }
        TimedPose ts = (TimedPose) other;
        boolean stateEqual = state().equals(ts.state());
        if (!stateEqual) {
            return false;
        }
        return Math.abs(getTimeS() - ts.getTimeS()) <= 1e-12;
    }
}
