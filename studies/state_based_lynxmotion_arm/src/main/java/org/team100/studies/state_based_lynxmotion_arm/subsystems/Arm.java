package org.team100.studies.state_based_lynxmotion_arm.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final Servo m_swing;
    private final Servo m_boom;
    private final Servo m_stick;
    private final Servo m_wrist;
    private final Servo m_twist;
    private final Servo m_grip;

    private static final double DEADBAND = 0.05;

    public Arm() {
        m_swing = new Servo(0);
        m_boom = new Servo(1);
        m_stick = new Servo(2);
        m_wrist = new Servo(3);
        m_twist = new Servo(4);
        m_grip = new Servo(5);
    }

    public Command goHome() {
        return run(this::home);
    }

    @Override
    public void periodic() {
        //
    }

    private void home() {
        m_swing.setAngle(90);
        m_boom.setAngle(135);
        m_stick.setAngle(90);
        m_wrist.setAngle(90);
        m_twist.setAngle(90); // not enough controller channels, leave twist fixed
        m_grip.setAngle(90);
    }
}
