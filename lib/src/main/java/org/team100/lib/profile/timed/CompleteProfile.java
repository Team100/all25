package org.team100.lib.profile.timed;

import org.team100.lib.profile.IncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

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
 */
public class CompleteProfile implements IncrementalProfile {

  
    @Override
    public Control100 calculate(double dt, Model100 setpoint, Model100 goal) {
        return new Control100();
    }

}
