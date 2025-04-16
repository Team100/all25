package org.team100.lib.encoder;

import java.util.OptionalDouble;

/**
 * Proxies an IncrementalBareEncoder to produce a RotaryPositionSensor, by
 * taking the angle modulus.
 * 
 * Use it with the CombinedRotaryPositionSensor.
 */
public class ProxyRotaryPositionSensor implements RotaryPositionSensor {
    private final IncrementalBareEncoder m_encoder;
    private final double m_gearRatio;

    public ProxyRotaryPositionSensor(IncrementalBareEncoder encoder, double gearRatio) {
        m_encoder = encoder;
        m_gearRatio = gearRatio;
    }

    /**
     * Sets the incremental encoder position. This is only used to "zero" it, and
     * only done by the CombinedRotaryPositionSensor.
     * 
     * It is very slow: call it only on startup.
     */
    public void setEncoderPosition(double positionRad) {
        double motorPositionRad = positionRad * m_gearRatio;
        m_encoder.setEncoderPositionRad(motorPositionRad);
    }

    /** Idential to RotaryMechanism.getPositionRad() */
    @Override
    public OptionalDouble getPositionRad() {
        OptionalDouble positionRad = m_encoder.getPositionRad();
        if (positionRad.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of(positionRad.getAsDouble() / m_gearRatio);
    }

    /**
     * Identical to RotaryMechanism.getVelocityRad_S()
     */
    @Override
    public OptionalDouble getVelocityRad_S() {
        OptionalDouble velocityRad_S = m_encoder.getVelocityRad_S();
        if (velocityRad_S.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of(velocityRad_S.getAsDouble() / m_gearRatio);
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public void close() {
        m_encoder.close();
    }

}
