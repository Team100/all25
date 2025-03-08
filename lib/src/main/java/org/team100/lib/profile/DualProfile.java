package org.team100.lib.profile;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * Contains two profiles: a "fast" one used far from the goal, and a "slow" one
 * used in the neighborhood of the goal.
 * 
 * The idea is for the "fast" profile to deliver the system to the "slow"
 * profile at its cruise velocity.
 */
public class DualProfile implements Profile100 {

    private final Profile100 m_fast;
    private final Profile100 m_slow;
    private final double m_neighborhood;

    public DualProfile(Profile100 fast, Profile100 slow, double neighborhood) {
        m_fast = fast;
        m_slow = slow;
        m_neighborhood = neighborhood;
    }

    @Override
    public Control100 calculate(double dt, Model100 initial, Model100 goal) {
        if (initial.near(goal, m_neighborhood, Double.MAX_VALUE)) {
            // near to the goal, use the slow profile
            return m_slow.calculate(dt, initial, goal);
        }
        // far from the goal
        // the goal velocity is the slow cruise velocity
        double goalV = m_slow.getMaxVelocity();
        // the goal position is one neighborhood away
        double goalX = goal.x() + Math.signum(initial.x() - goal.x()) * m_neighborhood;
        Model100 newGoal = new Model100(goalX, goalV);
        return m_fast.calculate(dt, initial, newGoal);
    }

    @Override
    public ResultWithETA calculateWithETA(double dt, Model100 initial, Model100 goal) {
        if (initial.near(goal, m_neighborhood, Double.MAX_VALUE)) {
            // near to the goal, use the slow profile
            return m_slow.calculateWithETA(dt, initial, goal);
        }
        // far from the goal
        // the goal velocity is the slow cruise velocity
        double goalV = m_slow.getMaxVelocity();
        // the goal position is one neighborhood away
        double goalX = goal.x() + Math.signum(initial.x() - goal.x()) * m_neighborhood;
        Model100 newGoal = new Model100(goalX, goalV);
        // add the two ETAs together
        ResultWithETA fastResult = m_fast.calculateWithETA(dt, initial, newGoal);
        ResultWithETA slowResult = m_slow.calculateWithETA(dt, newGoal, goal);
        return new ResultWithETA(fastResult.state(), fastResult.etaS() + slowResult.etaS());
    }

    @Override
    public DualProfile scale(double s) {
        return new DualProfile(m_fast.scale(s), m_slow.scale(s), m_neighborhood);
    }

    @Override
    public double getMaxVelocity() {
        // this is surely wrong
        return m_slow.getMaxVelocity();
    }

}
