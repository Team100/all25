package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;

/**
 * A calibratable servo, using an affine transfer function.
 * 
 * The WPI Servo class has the same sort of function inside, but it uses
 * constants (!). This class allows the transfer function to be specified.
 * 
 * Note that servos are proportional controllers and so they are "springy" in
 * the presence of load, i.e. the actual position will not, in general, be
 * exactly the commanded position.
 */
public class CalibratedServo implements AutoCloseable {
    /**
     * TODO: allow the pulse width range to be specified
     */
    private final Servo m_servo;

    /** Implements mechanical limits. */
    private final Clamp m_clamp;

    // TODO: are all the servos linear like this?
    /**
     * Transfer function from PWM to measured angle in radians. The calibration goes
     * from PWM to angle, instead of the reverse, which is what we actually want,
     * because servos don't have angle inputs or measurements.
     */
    private final AffineFunction m_transfer;

    /**
     * The actual servo, and the simulated servo, use integer pulse width
     * microseconds to record the setpoint; this number ranges from something like
     * 1000 to something like 2000, so the resolution is only 1000 steps or so. To
     * avoid surprising the caller we'll just remember the last thing they told us.
     */
    private double m_setpoint;

    public CalibratedServo(int channel, Clamp clamp, AffineFunction transfer) {
        m_servo = new Servo(channel);
        m_clamp = clamp;
        m_transfer = transfer;
    }

    /** Sets the angle in radians. */
    public void setAngle(double rad) {
        m_setpoint = rad;
        m_servo.set(m_transfer.x(m_clamp.f(rad)));
    }

    /** Returns the current setpoint (not the actual measurement) in radians. */
    public double getAngle() {
        double measurement = m_transfer.y(m_servo.get());
        if (!MathUtil.isNear(m_setpoint, measurement, 0.01)) {
            // if we're somehow way out of sync, then get back in sync.
            m_setpoint = measurement;
        }
        return m_setpoint;
    }

    @Override
    public void close() {
        m_servo.close();
    }

}
