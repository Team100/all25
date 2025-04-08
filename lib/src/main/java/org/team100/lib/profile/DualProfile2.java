package org.team100.lib.profile;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * For fast acceleration and slow deceleration.  Use for motionless goals.
 * 
 * This kinda works, but it seems lame.
 */
public class DualProfile2 implements Profile100 {
    private final Profile100 m_start;
    private final Profile100 m_end;

    public DualProfile2(Profile100 start, Profile100 end) {
        m_start = start;
        m_end = end;
    }

    @Override
    public Control100 calculate(double dt, Model100 setpoint, Model100 goal) {
        Control100 start = m_start.calculate(dt, setpoint, goal);
        Control100 end = m_end.calculate(dt, setpoint, goal);
        Model100 error = goal.minus(setpoint);
        if (Math.signum(setpoint.v()) != Math.signum(error.x())) {
            // we're moving away from the goal
            return start;
        }
        if (Math.signum(end.a()) == Math.signum(error.x())) {
            return start;
        }
        return end;
    }

    @Override
    public ResultWithETA calculateWithETA(double dt, Model100 initial, Model100 goal) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'calculateWithETA'");
    }

    @Override
    public Profile100 scale(double s) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'scale'");
    }

    @Override
    public double solve(double dt, Model100 i, Model100 g, double eta, double etaTolerance) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'solve'");
    }

    @Override
    public double getMaxVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMaxVelocity'");
    }
}
