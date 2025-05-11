package frc.robot;

import edu.wpi.first.wpilibj.Servo;

/**
 * A calibratable servo, using an affine transfer function.
 * 
 * The WPI Servo class has the same sort of function inside, but it uses
 * constants (!). This class allows the transfer function to be specified.
 */
public class CalibratedServo {
    private final Servo m_servo;
    // TODO: are all the servos linear like this?
    /** Transfer function from PWM to measured angle in radians. */
    private final AffineFunction m_transfer;

    public CalibratedServo(int channel, AffineFunction transfer) {
        m_servo = new Servo(channel);
        m_transfer = transfer;
    }

    /** Sets the angle in radians. */
    public void setAngle(double rad) {
        double pwm = m_transfer.x(rad);
        m_servo.set(pwm);
    }

}
