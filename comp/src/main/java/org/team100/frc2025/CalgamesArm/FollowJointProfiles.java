package org.team100.frc2025.CalgamesArm;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.Config;
import org.team100.lib.motion.kinematics.JointAccelerations;
import org.team100.lib.motion.kinematics.JointVelocities;
import org.team100.lib.profile.incremental.CompleteProfile;
import org.team100.lib.profile.incremental.CurrentLimitedExponentialProfile;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.MathUtil;

/**
 * Follow three uncoordinated profiles in configuration space.
 * Starting point and velocity are current measurements.
 */
public class FollowJointProfiles extends MoveAndHold {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;

    private final CalgamesMech m_subsystem;
    private final Model100 m_g1;
    private final Model100 m_g2;
    private final Model100 m_g3;
    private final IncrementalProfile m_p1;
    private final IncrementalProfile m_p2;
    private final IncrementalProfile m_p3;

    private Control100 m_c1;
    private Control100 m_c2;
    private Control100 m_c3;

    public static FollowJointProfiles WithCurrentLimitedExponentialProfile(
            CalgamesMech subsystem, Config goal) {
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CurrentLimitedExponentialProfile(2, 4, 5), // elevator
                new CurrentLimitedExponentialProfile(8, 8, 16), // arm
                new CurrentLimitedExponentialProfile(8, 8, 16)); // wrist
    }

    /**
     * Accelerate gently but decelerate firmly.
     * 
     * This is for paths that start with lots of gravity torque and end
     * without very much gravity torque, e.g. from "pick" to "home".
     */
    public static FollowJointProfiles slowFast(CalgamesMech subsystem, Config goal) {
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(2, 4, 6, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(12, 8, 16, 16, 50, 50, 0.001), // arm
                new CompleteProfile(8, 4, 12, 16, 50, 50, 0.001)); // wrist
    }

    /**
     * Accelerate firmly but decelerate gently.
     * 
     * This is for paths that start without gravity torque but end with a lot of
     * gravity torque, e.g. from "home" to "pick".
     */

    public static FollowJointProfiles fastSlow(CalgamesMech subsystem, Config goal) {
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(2, 6, 4, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(12, 20, 8, 16, 100, 100, 0.001), // arm
                new CompleteProfile(8, 12, 4, 16, 50, 50, 0.001)); // wrist
    }

    public static FollowJointProfiles algae(CalgamesMech subsystem, Config goal) {
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(2, 6, 4, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(12, 8, 8, 16, 100, 100, 0.001), // arm
                new CompleteProfile(8, 12, 4, 16, 50, 50, 0.001)); // wrist
    }
    public static FollowJointProfiles gentle(CalgamesMech subsystem, Config goal) {
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(2, 6, 4, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(2, 2, 2, 8, 100, 100, 0.001), // arm
                new CompleteProfile(8, 12, 4, 16, 50, 50, 0.001)); // wrist
    }

    public static FollowJointProfiles algaeUp(CalgamesMech subsystem, Config goal) {
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(2, 6, 4, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(4, 4, 4, 8, 100, 100, 0.001), // arm
                new CompleteProfile(8, 12, 4, 16, 50, 50, 0.001)); // wrist
    }


    FollowJointProfiles(
            CalgamesMech subsystem,
            Config goal,
            IncrementalProfile p1,
            IncrementalProfile p2,
            IncrementalProfile p3) {
        m_subsystem = subsystem;
        // Joint goals are motionless
        m_g1 = new Model100(goal.shoulderHeight(), 0);
        m_g2 = new Model100(goal.shoulderAngle(), 0);
        m_g3 = new Model100(goal.wristAngle(), 0);
        m_p1 = p1;
        m_p2 = p2;
        m_p3 = p3;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // initial position is current position
        Config c = m_subsystem.getConfig();
        // initial velocity is current velocity
        JointVelocities jv = m_subsystem.getJointVelocity();
        m_c1 = new Control100(c.shoulderHeight(), jv.elevator());
        m_c2 = new Control100(c.shoulderAngle(), jv.shoulder());
        m_c3 = new Control100(c.wristAngle(), jv.wrist());
    }

    @Override
    public void execute() {
        m_c1 = m_p1.calculate(DT, m_c1, m_g1);
        m_c2 = m_p2.calculate(DT, m_c2, m_g2);
        m_c3 = m_p3.calculate(DT, m_c3, m_g3);
        Config c = new Config(m_c1.x(), m_c2.x(), m_c3.x());
        JointVelocities jv = new JointVelocities(m_c1.v(), m_c2.v(), m_c3.v());
        JointAccelerations ja = new JointAccelerations(m_c1.v(), m_c2.v(), m_c3.v());
        m_subsystem.set(c, jv, ja);
    }

    @Override
    public boolean isDone() {
        return profileDone() && atReference();
    }

    /** The profile has reached the goal. */
    private boolean profileDone() {
        return MathUtil.isNear(m_g1.x(), m_c1.x(), 0.01)
                && MathUtil.isNear(m_g2.x(), m_c2.x(), 0.02)
                && MathUtil.isNear(m_g3.x(), m_c3.x(), 0.02)
                && Math.abs(m_c1.v()) < 0.01
                && Math.abs(m_c2.v()) < 0.02
                && Math.abs(m_c3.v()) < 0.02;

    }

    /** The measurement has reached the goal. */
    private boolean atReference() {
        Config c = m_subsystem.getConfig();
        JointVelocities jv = m_subsystem.getJointVelocity();
        return MathUtil.isNear(m_g1.x(), c.shoulderHeight(), 0.01)
                && MathUtil.isNear(m_g2.x(), c.shoulderAngle(), 0.02)
                && MathUtil.isNear(m_g3.x(), c.wristAngle(), 0.02)
                && Math.abs(jv.elevator()) < 0.01
                && Math.abs(jv.shoulder()) < 0.02
                && Math.abs(jv.wrist()) < 0.02;
    }

}
