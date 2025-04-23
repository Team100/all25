package org.team100.studies.state_based_lynxmotion_arm.motion;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Servo;

/** Move a servo with velocity/acceleration constraints. */
public class ProfiledServo {
    private static final double kPositionTolerance = 0.01;
    private static final double kVelocityTolerance = 0.01;

    private final TrapezoidProfile.Constraints m_maxConstraints;
    private final Servo m_servo;
    private final double m_min;
    private final double m_max;
    private final double m_maxVelocity;
    private final double m_maxAcceleration;

    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;

    public ProfiledServo(
            String name,
            int channel,
            double initialGoal,
            double min,
            double max,
            double maxVel,
            double maxAcc) {
        m_servo = new Servo(channel);
        m_min = min;
        m_max = max;
        m_maxVelocity = maxVel;
        m_maxAcceleration = maxAcc;
        m_maxConstraints = new TrapezoidProfile.Constraints(m_maxVelocity, m_maxAcceleration);

        setGoal(initialGoal, 1);
        m_setpoint = new TrapezoidProfile.State(initialGoal, 0);
        // m_constraints = m_maxConstraints; // in case we move before setting a goal
    }

    /*
     * Sets the goal, with [0,1] scaled velocity/acceleration, to coordinate with
     * other axes. Goal velocity is always zero.
     * TODO: allow nonzero velocity for better sequence linking
     */
    public void setGoal(double goal, double scale) {
        m_goal = new TrapezoidProfile.State(MathUtil.clamp(goal, m_min, m_max), 0);
        m_constraints = new TrapezoidProfile.Constraints(scale * m_maxConstraints.maxVelocity,
                scale * m_maxConstraints.maxAcceleration);
    }

    /** Returns the time the move would take at full speed. */
    public double eta(double goal) {
        TrapezoidProfile x = new TrapezoidProfile(m_maxConstraints);
        x.calculate(0.02, new TrapezoidProfile.State(goal, 0), m_setpoint); // this is awful
        return x.totalTime();
    }

    /** Calculates the new setpoint and moves the servo. */
    public void move(double dt) {
        var profile = new TrapezoidProfile(m_constraints);
        m_setpoint = profile.calculate(dt, m_goal, m_setpoint);
        m_servo.set(m_setpoint.position);
    }

    /** Returns position from [0,1]. */
    public double getPosition() {
        // TODO: use the actual position (which does not work in simulation)
        // return m_servo.get();
        return m_setpoint.position;
    }

    /** True if goal and setpoint are roughly equal. */
    public boolean atGoal() {
        return Math.abs(m_goal.position - m_setpoint.position) < kPositionTolerance &&
                Math.abs(m_goal.velocity - m_setpoint.velocity) < kVelocityTolerance;
    }

    /** TODO: remove */
    public double getAngle() {
        return m_servo.getAngle();
    }

    /** TODO: remove */
    public void setAngle(double x) {
        m_servo.setAngle(x);
        ;
    }
}