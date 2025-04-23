package org.team100.studies.state_based_lynxmotion_arm.subsystems;

import static edu.wpi.first.math.MathUtil.isNear;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private record Configuration(double swing, double boom, double stick, double wrist, double twist, double grip) {
    }

    private static final Configuration HOME = new Configuration(90, 135, 90, 90, 90, 90);
    private static final Configuration AWAY = new Configuration(135, 135, 90, 90, 90, 90);

    private static final double TOLERANCE = 0.05;
    private final Servo m_swing;
    private final Servo m_boom;
    private final Servo m_stick;
    private final Servo m_wrist;
    private final Servo m_twist;
    private final Servo m_grip;

    private final Timer m_awayTimer;

    public Arm() {
        m_swing = new Servo(0);
        m_boom = new Servo(1);
        m_stick = new Servo(2);
        m_wrist = new Servo(3);
        m_twist = new Servo(4);
        m_grip = new Servo(5);

        m_awayTimer = new Timer();
    }

    // OBSERVATIONS

    public boolean isHome() {
        return isAt(HOME);
    }

    public boolean isAway() {
        return isAt(AWAY);
    }

    public boolean awayTimerExpired() {
        return m_awayTimer.hasElapsed(1);
    }

    // ACTIONS

    public Command goHome() {
        return run(this::home);
    }

    public Command goAway() {
        return startRun(m_awayTimer::restart, this::away);
    }

    @Override
    public void periodic() {
        //
    }

    //////////////////////////////////////////////////

    private void home() {
        set(HOME);
    }

    private void away() {
        set(AWAY);
    }

    private boolean isAt(Configuration target) {
        return isNear(m_swing.getAngle(), target.swing, TOLERANCE)
                && isNear(m_boom.getAngle(), target.boom, TOLERANCE)
                && isNear(m_stick.getAngle(), target.stick, TOLERANCE)
                && isNear(m_wrist.getAngle(), target.wrist, TOLERANCE)
                && isNear(m_twist.getAngle(), target.twist, TOLERANCE)
                && isNear(m_grip.getAngle(), target.grip, TOLERANCE);
    }

    private void set(Configuration target) {
        m_swing.setAngle(target.swing);
        m_boom.setAngle(target.boom);
        m_stick.setAngle(target.stick);
        m_wrist.setAngle(target.wrist);
        m_twist.setAngle(target.twist);
        m_grip.setAngle(target.grip);
    }
}
