package org.team100.lib.profile.timed;

import org.team100.lib.profile.IncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * A simple profile to implement all the things we want from a
 * motion profile for mechanisms:
 * 
 * * current limit
 * * back-EMF limit
 * * jerk-limited tapering
 * 
 * So there are five parameters:
 * 
 * * maximum acceleration
 * * maximum deceleration (typically higher than accel, "plugging")
 * * maximum velocity
 * * "stall" acceleration for calculating back EMF
 * * maximum jerk, for tapering
 * 
 * the initial state can be anything.
 * 
 * the goal is always stationary.
 * 
 * This works by precalculating the "goal" path on instantiation, and
 * calculating the initial path dynamically. when the initial state is close to
 * the goal path, then states are interpolated from it.
 * 
 * so this is actually an incremental profile.
 * 
 * Note that only stationary goal states are allowed, so that the goal path can
 * be precalculated (and because 100% of our actual paths have stationary
 * goals).
 * 
 * It's similar to a sliding-mode controller: maximum effort to reach a sliding
 * manifold, and then moving alnog it.
 */
public class CompleteProfile implements IncrementalProfile {
    InterpolatingTreeMap<Double, Control100> m_byDistance;

    public CompleteProfile() {
        // InverseInterpolator<Double> keyInterpolator = InverseInterpolator.forDouble();
        // Interpolator<Control100> valueInterpolator = 
        // m_byDistance = new InterpolatingTreeMap<>();
    }

    @Override
    public Control100 calculate(double dt, Model100 setpoint, Model100 goal) {
        if (Math.abs(goal.v()) > 1e-6)
            throw new IllegalArgumentException("This profile works only with stationary goals.");
        double togo = goal.x() - setpoint.x();
        Control100 lerp = m_byDistance.get(togo);
        // if (setpoint.v())

            return new Control100();
    }

}
