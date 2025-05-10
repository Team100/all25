package frc.robot;

import edu.wpi.first.wpilibj.Servo;

public class CalibratedServo {
    private final Servo m_servo;

    public CalibratedServo(int channel) {
        m_servo = new Servo(channel);
    }

}
