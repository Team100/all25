package org.team100.lib.profile.timed;

import org.team100.lib.spline.SepticSpline1d;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

/*************************
 * DO NOT USE
 *************************/
public class SepticSplineProfile implements TimedProfile {
    private static final boolean DEBUG = true;

    private final double vel;
    private final double acc;

    SepticSpline1d m_spline;
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

        double adjustedAcceleration = acc;

        if (DEBUG)
            Util.printf("SepticSplineProfile init initial %s goal %s\n", initial, goal);

        double x0 = initial.x();
        double x1 = goal.x();
        double dx0 = initial.v();
        double dx1 = goal.v();
        double ddx0 = 0;
        double ddx1 = 0;
        double dddx0 = 0;
        double dddx1 = 0;

        init(x0, x1, dx0, dx1, ddx0, ddx1, dddx0, dddx1);
    }

    // these are spline units not profile units.
    // we don't know the profile units until we compute the duration.
    // TODO: just specify the duration, since the spline doesn't
    // let you adjust the maxima independently anyway.
    void init(
            double x0, double x1,
            double dx0, double dx1,
            double ddx0, double ddx1,
            double dddx0, double dddx1) {
        m_spline = SepticSpline1d.viaMatrix(
                x0, x1,
                dx0, dx1,
                ddx0, ddx1, dddx0, dddx1);
        double vScale = m_spline.maxV / vel;
        double aScale = Math.sqrt(m_spline.maxA) / Math.sqrt(acc);
        m_duration = Math.max(vScale, aScale);
        if (Double.isNaN(m_duration))
            m_duration = 0;
        if (DEBUG) {
            // Util.printf("SepticSplineProfile init duration %f\n", m_duration);
            // Util.printf("sample 0 %s 1 %s\n", m_spline.getPosition(0),
            // m_spline.getPosition(1));
        }

    }

    @Override
    public Control100 sample(double timeS) {
        if (m_duration < 1e-6) {
            // zero duration, we're always at the end.
            double x = m_spline.getPosition(1);
            double v = m_spline.getVelocity(1);
            double a = m_spline.getAcceleration(1);
            return new Control100(x, v, a);
        }
        double param = getParam(timeS);
        double x = m_spline.getPosition(param);
        double v = m_spline.getVelocity(param) / m_duration;
        double a = m_spline.getAcceleration(param) / (m_duration * m_duration);
        if (DEBUG)
            Util.printf("%12.6f  %12.6f  %12.6f  %12.6f\n", timeS, x, v, a);

        return new Control100(x, v, a);
    }

    private double getParam(double timeS) {
        if (timeS > m_duration)
            timeS = m_duration;
        return timeS / m_duration;
    }

    @Override
    public double duration() {
        return m_duration;
    }

}
