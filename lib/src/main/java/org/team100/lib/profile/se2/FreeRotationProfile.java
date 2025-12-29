package org.team100.lib.profile.se2;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.Model100;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.MathUtil;

/**
 * Like HolonomicProfile, but only coordinates X and Y, leaving theta free.
 * 
 * This is useful for cases where the robot rotation should be faster than the
 * translation, e.g. to make sure the robot arrives with the correct
 * orientation.
 */
public class FreeRotationProfile implements ProfileSE2 {

    /** For testing */
    private static final boolean DEBUG = false;
    /** Solver accuracy is low, in the interest of speed. */
    private static final double ETA_TOLERANCE = 0.1;
    /** Simulation for ETA is coarse, in the interest of speed. */
    private static final double SOLVE_DT = 0.1;
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;

    private final IncrementalProfile px;
    private final IncrementalProfile py;
    private final IncrementalProfile ptheta;

    // package-private for testing
    IncrementalProfile ppx;
    IncrementalProfile ppy;
    // for testing only
    double sx;
    double sy;

    FreeRotationProfile(
            IncrementalProfile px,
            IncrementalProfile py,
            IncrementalProfile ptheta) {
        this.px = px;
        this.py = py;
        this.ptheta = ptheta;
        ppx = px;
        ppy = py;
    }

    /**
     * This is a fairly coarse optimization, i.e. ETA within 0.1 sec or so.
     */
    @Override
    public void solve(ModelSE2 i, ModelSE2 g) {
        // first find the max ETA
        if (DEBUG) {
            System.out.printf("i %s g %s\n", i, g);
        }
        // note coarser DT
        double xETA = px.simulateForETA(SOLVE_DT, i.x().control(), g.x());
        double yETA = py.simulateForETA(SOLVE_DT, i.y().control(), g.y());

        if (DEBUG) {
            System.out.printf("ETAs: %f %f\n", xETA, yETA);
        }
        double slowETA = xETA;
        slowETA = Math.max(slowETA, yETA);

        sx = px.solve(SOLVE_DT, i.x().control(), g.x(), slowETA, ETA_TOLERANCE);
        sy = py.solve(SOLVE_DT, i.y().control(), g.y(), slowETA, ETA_TOLERANCE);

        if (DEBUG) {
            System.out.printf("sx %.3f sy %.3f\n", sx, sy);
        }

        ppx = px.scale(sx);
        ppy = py.scale(sy);
    }

    @Override
    public ControlSE2 calculate(ModelSE2 i, ModelSE2 g) {
        if (i == null || g == null) {
            // this can happen on startup when the initial state hasn't yet been defined,
            // but the cache refresher is trying to update the references.
            return ControlSE2.zero();
        }
        if (DEBUG) {
            System.out.printf("initial %s goal %s\n", i, g);
        }
        Control100 stateX = ppx.calculate(DT, i.x().control(), g.x());
        Control100 stateY = ppy.calculate(DT, i.y().control(), g.y());
        // theta is periodic; choose a setpoint angle near the goal.
        Model100 theta = new Model100(
                MathUtil.angleModulus(i.theta().x() - g.theta().x()) + g.theta().x(),
                i.theta().v());
        Control100 stateTheta = ptheta.calculate(DT, theta.control(), g.theta());
        return new ControlSE2(stateX, stateY, stateTheta);
    }
}
