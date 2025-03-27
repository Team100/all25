package org.team100.lib.profile.timed;

import org.team100.lib.spline.SepticSpline1d;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

public class SepticSplineProfile implements TimedProfile {
    private static final boolean DEBUG = true;

    private final double vel;
    private final double acc;

    private SepticSpline1d m_spline;
    private double m_duration;

    /**
     * Specify velocity and acceleration limits.
     * The initial and final accelerations and jerks are zero.
     */
    public SepticSplineProfile(double vel, double acc) {
        this.vel = vel;
        this.acc = acc;
    }

    @Override
    public void init(Model100 initial, Model100 goal) {
        if (DEBUG)
            Util.printf("INIT initial %s goal %s\n", initial, goal);
        m_spline = SepticSpline1d.viaMatrix(
                initial.x(), goal.x(),
                initial.v(), goal.v(),
                0, 0, 0, 0);
        double vScale = m_spline.maxV / vel;
        double aScale = Math.sqrt(m_spline.maxA) / Math.sqrt(acc);
        m_duration = Math.max(vScale, aScale);
    }

    @Override
    public Control100 sample(double timeS) {
        if (timeS > m_duration)
            timeS = m_duration;
        double param = timeS / m_duration;
        double x = m_spline.getPosition(param);
        double v = m_spline.getVelocity(param) / m_duration;
        double a = m_spline.getAcceleration(param) / (m_duration * m_duration);
        return new Control100(x, v, a);
    }

    @Override
    public double duration() {
        return m_duration;
    }

}
