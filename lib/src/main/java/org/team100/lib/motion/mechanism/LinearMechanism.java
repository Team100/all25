package org.team100.lib.motion.mechanism;

import java.util.OptionalDouble;

import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.motor.BareMotor;

/**
 * Uses a motor, gears, and a wheel to produce linear output, e.g. a drive wheel
 * or conveyor belt.
 * 
 * The limits used to be enforced by a proxy, but now they're here: it seems
 * simpler that way.
 */
public class LinearMechanism {
    private final BareMotor m_motor;
    private final IncrementalBareEncoder m_encoder;
    private final double m_gearRatio;
    private final double m_wheelRadiusM;
    private final double m_minPositionM;
    private final double m_maxPositionM;

    public LinearMechanism(
            BareMotor motor,
            IncrementalBareEncoder encoder,
            double gearRatio,
            double wheelDiameterM,
            double minPositionM,
            double maxPositionM) {
        m_motor = motor;
        m_encoder = encoder;
        m_gearRatio = gearRatio;
        m_wheelRadiusM = wheelDiameterM / 2;
        m_minPositionM = minPositionM;
        m_maxPositionM = maxPositionM;
    }

    /** Should actuate immediately.  Use for homing. */
    public void setDutyCycleUnlimited(double output) {
        m_motor.setDutyCycle(output);
    }

    /** Should actuate immediately. Limits position using the encoder. */
    public void setDutyCycle(double output) {
        OptionalDouble posOpt = getPositionM();
        if (posOpt.isEmpty()) {
            m_motor.stop();
            return;
        }
        double posM = posOpt.getAsDouble();
        if (output < 0 && posM < m_minPositionM) {
            m_motor.stop();
            return;
        }
        if (output > 0 && posM > m_maxPositionM) {
            m_motor.stop();
            return;
        }
        m_motor.setDutyCycle(output);
    }

    public void setForceLimit(double forceN) {
        m_motor.setTorqueLimit(forceN * m_wheelRadiusM / m_gearRatio);
    }

    /** Should actuate immediately. Use for homing. */
    public void setVelocityUnlimited(double outputVelocityM_S, double outputAccelM_S2, double outputForceN) {
        m_motor.setVelocity(
                (outputVelocityM_S / m_wheelRadiusM) * m_gearRatio,
                (outputAccelM_S2 / m_wheelRadiusM) * m_gearRatio,
                outputForceN * m_wheelRadiusM / m_gearRatio);
    }

    /** Should actuate immediately. Limits position using the encoder. */
    public void setVelocity(
            double outputVelocityM_S,
            double outputAccelM_S2,
            double outputForceN) {
        OptionalDouble posOpt = getPositionM();
        if (posOpt.isEmpty()) {
            m_motor.stop();
            return;
        }
        double posM = posOpt.getAsDouble();
        if (outputVelocityM_S < 0 && posM < m_minPositionM) {
            m_motor.stop();
            return;
        }
        if (outputVelocityM_S > 0 && posM > m_maxPositionM) {
            m_motor.stop();
            return;
        }
        m_motor.setVelocity(
                (outputVelocityM_S / m_wheelRadiusM) * m_gearRatio,
                (outputAccelM_S2 / m_wheelRadiusM) * m_gearRatio,
                outputForceN * m_wheelRadiusM / m_gearRatio);
    }

    /** Should actuate immediately. Limits position using the argument to this function. */
    public void setPosition(
            double outputPositionM,
            double outputVelocityM_S,
            double outputAccelM_S2,
            double outputForceN) {
        if (outputPositionM < m_minPositionM) {
            m_motor.stop();
            return;
        }
        if (outputPositionM > m_maxPositionM) {
            m_motor.stop();
            return;
        }
        m_motor.setPosition(
                (outputPositionM / m_wheelRadiusM) * m_gearRatio,
                (outputVelocityM_S / m_wheelRadiusM) * m_gearRatio,
                (outputAccelM_S2 / m_wheelRadiusM) * m_gearRatio,
                outputForceN * m_wheelRadiusM / m_gearRatio);
    }

    /** Nearly cached. */
    public OptionalDouble getVelocityM_S() {
        OptionalDouble velocityRad_S = m_encoder.getVelocityRad_S();
        if (velocityRad_S.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of(velocityRad_S.getAsDouble() * m_wheelRadiusM / m_gearRatio);
    }

    /** Nearly cached. */
    public OptionalDouble getPositionM() {
        OptionalDouble positionRad = m_encoder.getPositionRad();
        if (positionRad.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of(positionRad.getAsDouble() * m_wheelRadiusM / m_gearRatio);
    }

    public void stop() {
        m_motor.stop();
    }

    public void close() {
        m_motor.close();
        m_encoder.close();
    }

    /**
     * Caches should also be flushed, so the new value is available immediately.
     */
    public void resetEncoderPosition() {
        m_encoder.reset();
    }

    public void periodic() {
        // do some logging
        m_motor.periodic();
        m_encoder.periodic();
    }

}
