package org.team100.lib.profile.timed;

import org.team100.lib.spline.SepticSpline1d;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

public class SepticSplineProfile implements TimedProfile {
    private final double vel;
    private final double acc;

    private SepticSpline1d spline;
    double duration;

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
        spline = SepticSpline1d.viaMatrix(
                initial.x(), goal.x(),
                initial.v(), goal.v(),
                0, 0, 0, 0);
        double vScale = spline.maxV / vel;
        double aScale = Math.sqrt(spline.maxA) / Math.sqrt(acc);
        duration = Math.max(vScale, aScale);
    }

    @Override
    public Control100 sample(double timeS) {
        if (timeS > duration)
            timeS = duration;
        double param = timeS / duration;
        double x = spline.getPosition(param);
        double v = spline.getVelocity(param) / duration;
        double a = spline.getAcceleration(param) / (duration * duration);
        return new Control100(x, v, a);

    }

}
