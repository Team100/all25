package org.team100.lib.profile.r1;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;

public class ExponentialProfileWPI implements IncrementalProfile {
    private final Constraints m_constraints;
    private final ExponentialProfile m_profile;

    public ExponentialProfileWPI(double maxVel, double maxAccel) {
        // The WPI class uses unfamiliar notation:
        // a = Av + Bu
        // A is a negative number representing back EMF (a opposes v)
        // B is a positive number representing torque (u produces a)
        // Max stall acceleration (v=0) = B * maxU
        // Max velocity (a=0) = -B * maxU / A.
        // So if max U is [0,1] then
        // B = max accel.
        // A = - max accel divided by max velocity.
        double A = -1.0 * maxAccel / maxVel;
        double B = maxAccel;
        m_constraints = Constraints.fromStateSpace(1, A, B);
        m_profile = new ExponentialProfile(m_constraints);
    }

    @Override
    public Control100 calculate(double dt, Control100 initial, Model100 goal) {
        State result = m_profile.calculate(dt, new State(initial.x(), initial.v()), new State(goal.x(), goal.v()));
        return new Control100(result.position, result.velocity, 0);
    }

    @Override
    public IncrementalProfile scale(double s) {
        return new ExponentialProfileWPI(
                m_constraints.maxVelocity(), s * m_constraints.B);
    }

    public double getMaxVelocity() {
        return m_constraints.maxVelocity();
    }
}
