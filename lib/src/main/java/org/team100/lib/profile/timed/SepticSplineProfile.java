package org.team100.lib.profile.timed;

import org.team100.lib.spline.SepticSpline1d;
import org.team100.lib.spline.SepticSpline1d.SplineException;
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

        double adjustedAcceleration = acc;

        if (DEBUG)
            Util.printf("SepticSplineProfile init initial %s goal %s\n", initial, goal);

        if(goal.x() < initial.x()) {
            adjustedAcceleration *= -1;
        }
        m_spline = SepticSpline1d.viaMatrix(
                initial.x(), goal.x(),
                initial.v(), goal.v(),
                adjustedAcceleration, 0, 0, 0); //CHANE TO ACC
        double vScale = m_spline.maxV / vel;
        double aScale = Math.sqrt(m_spline.maxA) / Math.sqrt(acc);
        m_duration = Math.max(vScale, aScale);
        if (Double.isNaN(m_duration))
            m_duration = 0;
        if (DEBUG)
            Util.printf("SepticSplineProfile init duration %f\n", m_duration);
        if (DEBUG)
            Util.printf("sample 0 %s 1 %s\n", m_spline.getPosition(0), m_spline.getPosition(1));
            System.out.println("V SCALE " + vScale);
            System.out.println("A SCALE " + aScale);
            System.out.println("MAX SPLINE A" + m_spline.maxA);

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
        double v = m_duration < 1e-6 ? m_spline.getVelocity(param) : m_spline.getVelocity(param) / m_duration;
        double a = m_spline.getAcceleration(param) / (m_duration * m_duration);
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
