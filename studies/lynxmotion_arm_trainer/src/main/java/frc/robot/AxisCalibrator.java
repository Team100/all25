package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Move one axis at a time. Moves from 0.0 to 1.0 in steps of 0.1. */
public class AxisCalibrator extends SubsystemBase {
    private final Servo m_servo;

    private double m_position = 0.0;
    private double m_inc = 0.1;

    public AxisCalibrator(int channel) {
        m_servo = new Servo(channel);
    }

    public double getPosition() {
        return m_position;
    }

    public Command step() {
        return runOnce(this::doStep);
    }

    // TODO: put this in a separate class.
    public void doStep() {
        if (m_position + m_inc >= 0.95) {
            // we're at the end, so go back
            m_inc = -0.1;
            m_position = 1.0;
        } else if (m_position + m_inc <= 0.05) {
            // we're at the start, so go forward
            m_inc = 0.1;
            m_position = 0.0;
        } else {
            m_position += m_inc;
        }
        System.out.printf("position %f\n", m_position);
        m_servo.set(m_position);
    }

}
