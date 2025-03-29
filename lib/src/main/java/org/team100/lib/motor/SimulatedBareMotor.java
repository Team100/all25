package org.team100.lib.motor;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.spline.SepticSpline1d.SplineException;
import org.team100.lib.util.Memo;
import org.team100.lib.util.Memo.DoubleCache;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;

public class SimulatedBareMotor implements BareMotor {
    private static final boolean DEBUG = false;

    private final double m_freeSpeedRad_S;

    private final DoubleLogger m_log_duty;
    private final DoubleLogger m_log_velocity;
    private final DoubleCache m_velocityCache;
    private final DoubleCache m_positionCache;

    private double m_velocity = 0;
    private double m_position = 0;
    private double m_time = Takt.get();

    public SimulatedBareMotor(LoggerFactory parent, double freeSpeedRad_S) {
        LoggerFactory child = parent.child(this);
        m_freeSpeedRad_S = freeSpeedRad_S;
        m_log_duty = child.doubleLogger(Level.DEBUG, "duty_cycle");
        m_log_velocity = child.doubleLogger(Level.DEBUG, "velocity (rad_s)");
        m_velocityCache = Memo.ofDouble(() -> m_velocity);
        m_positionCache = Memo.ofDouble(() -> m_position);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        final double output = MathUtil.clamp(
                Util.notNaN(dutyCycle), -1, 1);
        m_log_duty.log(() -> output);
        setVelocity(output * m_freeSpeedRad_S, 0, 0);
    }

    /** ignores accel and torque */
    @Override
    public void setVelocity(double velocityRad_S, double accelRad_S2, double torqueNm) {
        m_velocity = MathUtil.clamp(
                Util.notNaN(velocityRad_S), -m_freeSpeedRad_S, m_freeSpeedRad_S);
        m_log_velocity.log(() -> m_velocity);

        // this is also in the simulated encoder
        double now = Takt.get();
        double dt = now - m_time;
        if (dt < 1e-6) {
            // calling twice in the same cycle => nothing happens
            return;
        }
        if (DEBUG)
            Util.printf("motor set pos %f v %f dt %f now %f m_time %f\n", m_position, velocityRad_S, dt, now, m_time);
        m_position += velocityRad_S * dt;
        m_time = now;

    }

    /** ignores velocity and torque */
    @Override
    public void setPosition(double position, double velocity, double accel, double torque) {
        double now = Takt.get();
        double dt = now - m_time;
        if (dt < 1e-6) {
            // calling twice in the same cycle => nothing happens
            return;
        }
        double dx = position - m_position;
        m_velocity = dx / dt;
        m_position = position;
        m_time = now;
        if (DEBUG)
            Util.printf("set motor position %.6f\n", m_position);
    }

    /** placeholder */
    @Override
    public double kROhms() {
        return 0.1;
    }

    /** placeholder */
    @Override
    public double kTNm_amp() {
        return 0.02;
    }

    @Override
    public void stop() {
        m_velocity = 0;
    }

    @Override
    public void close() {
        //
    }

    @Override
    public double getVelocityRad_S() {
        return m_velocityCache.getAsDouble();
    }

    public double getPositionRad() {
        double pos = m_positionCache.getAsDouble();
        if (Double.isNaN(pos))
            throw new SplineException("motor pos");
        return pos;
    }

    /** resets the caches, so the new value is immediately available. */
    @Override
    public void setEncoderPositionRad(double positionRad) {
        if (Double.isNaN(positionRad))
            throw new SplineException("motor set position");
        m_position = positionRad;
        m_positionCache.reset();
        m_velocityCache.reset();
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        //
    }

    @Override
    public void periodic() {
        //
    }

    /** resets the caches, so the new value is immediately available. */
    public void reset() {
        m_position = 0;
        m_time = Takt.get();
        m_positionCache.reset();
        m_velocityCache.reset();
    }
}
