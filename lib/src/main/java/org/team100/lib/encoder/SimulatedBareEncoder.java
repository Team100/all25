package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.OptionalDoubleLogger;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.util.Util;

public class SimulatedBareEncoder implements IncrementalBareEncoder {
    private static final boolean DEBUG = false;
    private final SimulatedBareMotor m_motor;

    private OptionalDoubleLogger m_log_position;
    private OptionalDoubleLogger m_log_velocity;

    public SimulatedBareEncoder(
            LoggerFactory parent,
            SimulatedBareMotor motor) {
        LoggerFactory child = parent.child(this);
        m_motor = motor;
        m_log_position = child.optionalDoubleLogger(Level.TRACE, "position (rad)");
        m_log_velocity = child.optionalDoubleLogger(Level.TRACE, "velocity (rad_s)");
    }

    /** Value should be updated in Robot.robotPeriodic(). */
    @Override
    public OptionalDouble getVelocityRad_S() {
        return OptionalDouble.of(m_motor.getVelocityRad_S());
    }

    /**
     * Value should be updated in Robot.robotPeriodic().
     * 
     * Derives position by integrating velocity over one time step.
     */
    @Override
    public OptionalDouble getPositionRad() {
        double positionRad = m_motor.getPositionRad();
        if (DEBUG)
            Util.printf("read encoder position %.6f\n", positionRad);
        return OptionalDouble.of(positionRad);
    }

    @Override
    public void reset() {
        m_motor.reset();
    }

    @Override
    public void close() {
        //
    }

    @Override
    public void setEncoderPositionRad(double motorPositionRad) {
        m_motor.setEncoderPositionRad(motorPositionRad);
    }

    @Override
    public void periodic() {
        m_log_position.log(this::getPositionRad);
        m_log_velocity.log(this::getVelocityRad_S);
    }

    @Override
    public double getPositionBlockingRad() {
        return getPositionRad().getAsDouble();
    }
}
