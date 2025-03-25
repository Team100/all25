package org.team100.lib.profile.timed;

import org.team100.lib.spline.SepticSpline1d;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

public class SepticSplineProfile implements TimedProfile {
    private final SepticSpline1d spline;
    final double duration;

    /** The initial and final accelerations and jerks are zero. */
    public SepticSplineProfile(Model100 initial, Model100 goal, double vel, double acc) {
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
