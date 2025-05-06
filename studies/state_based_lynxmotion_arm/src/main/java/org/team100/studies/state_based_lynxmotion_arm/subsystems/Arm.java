package org.team100.studies.state_based_lynxmotion_arm.subsystems;

import static edu.wpi.first.math.MathUtil.isNear;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.lynxmotion_arm.LynxArmAngles;
import org.team100.studies.state_based_lynxmotion_arm.motion.ProfiledServo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static final double TOLERANCE = 0.05;
    private static final double wristSpeed = 5;
    private static final double wristAccel = 5;
    private static final double speed = 1;
    private static final double accel = 1;

    private final LynxArmAngles HOME;
    private final LynxArmAngles AWAY;
    private final LynxArmAngles east;
    private final LynxArmAngles west;
    private final LynxArmAngles north;

    private final LynxArmAngles.Factory m_factory;

    private final ProfiledServo m_swing;
    private final ProfiledServo m_boom;
    private final ProfiledServo m_stick;
    private final ProfiledServo m_wrist;
    private final ProfiledServo m_twist;
    private final ProfiledServo m_grip;

    public Arm(LynxArmAngles.Factory factory) {
        m_factory = factory;
        LynxArmAngles initial = m_factory.fromRad(
                0, -Math.PI / 4, Math.PI / 2, Math.PI / 4, 0.5, 0.9);
        HOME = m_factory.fromDeg(0, 100, 90, 90, 0.5, 0.5);
        AWAY = m_factory.fromDeg(0, 0, 0, 0, 0, 0);
        east = m_factory.from0_1(0, 0.2, 0.35, 0.5, 0, 0);
        west = m_factory.from0_1(1.0, 0.2, 0.35, 0.5, 0, 0);
        north = m_factory.from0_1(0.5, 0.5, 0.8, 0.2, 0, 0);

        m_swing = new ProfiledServo("Swing", 0, initial.swing, 0, 1, speed, accel);
        m_boom = new ProfiledServo("Boom", 1, initial.boom, 0, 1, speed, accel);
        m_stick = new ProfiledServo("Stick", 2, initial.stick, 0, 1, speed, accel);
        m_wrist = new ProfiledServo("Wrist", 3, initial.wrist, 0, 1, wristSpeed, wristAccel);
        m_twist = new ProfiledServo("Twist", 4, initial.twist, 0, 1, 1, 1);
        m_grip = new ProfiledServo("Grip", 5, initial.grip, 0, 1, 1, 1);
    }

    // OBSERVATIONS

    public boolean isHome() {
        return isAt(HOME);
    }

    public boolean isAway() {
        return isAt(AWAY);
    }

    public boolean isWest() {
        return isAt(west);
    }

    public boolean isEast() {
        return isAt(east);
    }

    public boolean isNorth() {
        return isAt(north);
    }

    // ACTIONS

    // TODO: these should probably reset the profiles in all the axes, otherwise
    // nothing ever does.

    public Command goHome() {
        return run(() -> setRawGoals(HOME)).withName("go home");
    }

    public Command goAway() {
        return run(() -> setRawGoals(AWAY)).withName("go away");
    }

    public Command goWest() {
        return run(() -> setRawGoals(west));
    }

    public Command goEast() {
        return run(() -> setRawGoals(east));
    }

    public Command goNorth() {
        return run(() -> setRawGoals(north));
    }

    @Override
    public void periodic() {
        //
    }

    public LynxArmAngles get() {
        return m_factory.from0_1(
                m_swing.getPosition(),
                m_boom.getPosition(),
                m_stick.getPosition(),
                m_wrist.getPosition(),
                m_twist.getPosition(),
                m_grip.getPosition());
    }

    public void move(double dt) {
        m_swing.move(dt);
        m_boom.move(dt);
        m_stick.move(dt);
        m_wrist.move(dt);
        m_twist.move(dt);
        m_grip.move(dt);
    }

    //////////////////////////////////////////////////

    private boolean isAt(LynxArmAngles target) {
        boolean swing = isNear(m_swing.getPosition(), target.swing, TOLERANCE);
        boolean boom = isNear(m_boom.getPosition(), target.boom, TOLERANCE);
        boolean stick = isNear(m_stick.getPosition(), target.stick, TOLERANCE);
        boolean wrist = isNear(m_wrist.getPosition(), target.wrist, TOLERANCE);
        boolean twist = isNear(m_twist.getPosition(), target.twist, TOLERANCE);
        boolean grip = isNear(m_grip.getPosition(), target.grip, TOLERANCE);
        return swing && boom && stick && wrist && twist && grip;
    }

    private void set(LynxArmAngles target) {
        m_swing.setAngle(target.swing);
        m_boom.setAngle(target.boom);
        m_stick.setAngle(target.stick);
        m_wrist.setAngle(target.wrist);
        m_twist.setAngle(target.twist);
        m_grip.setAngle(target.grip);
    }

    /*
     * Applies the specified goals, adjusting velocity/acceleration so that all the
     * axes will complete at the same time.
     * 
     * TODO: add boundary enforcement (e.g. raise boom to avoid hitting the table).
     */
    private void setGoals(LynxArmAngles goals) {
        double slowest_eta = slowestEta(goals);
        setGoal(m_swing, goals.swing, slowest_eta);
        setGoal(m_boom, goals.boom, slowest_eta);
        setGoal(m_stick, goals.stick, slowest_eta);
        setGoal(m_wrist, goals.wrist, slowest_eta);
        setGoal(m_twist, goals.twist, slowest_eta);
        setGoal(m_grip, goals.grip, slowest_eta);
        move(TimedRobot100.LOOP_PERIOD_S);
    }

    private void setRawGoals(LynxArmAngles goals) {
        m_swing.setGoal(goals.swing, 1);
        m_boom.setGoal(goals.boom, 1);
        m_stick.setGoal(goals.stick, 1);
        m_wrist.setGoal(goals.wrist, 1);
        m_twist.setGoal(goals.twist, 1);
        m_grip.setGoal(goals.grip, 1);
        move(TimedRobot100.LOOP_PERIOD_S);
    }

    private void setGoal(ProfiledServo servo, double goal, double slowest_eta) {
        double eta = servo.eta(goal);
        servo.setGoal(goal, eta / slowest_eta);
    }

    /** Finds the slowest axis; use this to slow down the other axes. */
    private double slowestEta(LynxArmAngles goals) {
        double slowest_eta = 0;
        slowest_eta = Math.max(slowest_eta, m_swing.eta(goals.swing));
        slowest_eta = Math.max(slowest_eta, m_boom.eta(goals.boom));
        slowest_eta = Math.max(slowest_eta, m_stick.eta(goals.stick));
        slowest_eta = Math.max(slowest_eta, m_wrist.eta(goals.wrist));
        slowest_eta = Math.max(slowest_eta, m_twist.eta(goals.twist));
        slowest_eta = Math.max(slowest_eta, m_grip.eta(goals.grip));
        return slowest_eta;
    }
}
