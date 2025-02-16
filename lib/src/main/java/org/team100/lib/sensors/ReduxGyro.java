package org.team100.lib.sensors;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Rotation2dLogger;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroFaults;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import com.reduxrobotics.sensors.canandgyro.QuaternionFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Redux gyro folk say that the measurements provided are within 1ms of the
 * CAN packet being sent, and the CAN latency is quite low, but the CAN packet
 * might be received sometime before the RoboRIO interrupt fires; the logic
 * here corrects for that difference.
 */
public class ReduxGyro implements Gyro {

    private final Canandgyro m_gyro;

    // LOGGERS
    private final Rotation2dLogger m_log_yaw;
    private final DoubleLogger m_log_yaw_rate;
    private final Rotation2dLogger m_log_pitch;
    private final Rotation2dLogger m_log_roll;

    public ReduxGyro(LoggerFactory parent, int canID) {
        LoggerFactory child = parent.child(this);
        m_gyro = new Canandgyro(canID);
        m_gyro.clearStickyFaults();

        // Both position and velocity should be reasonably fresh.
        CanandgyroSettings settings = new CanandgyroSettings();
        settings.setAngularPositionFramePeriod(0.01);
        settings.setAngularVelocityFramePeriod(0.01);
        if (!m_gyro.setSettings(settings, 0.1)) {
            Util.warn("!!!!!!!!!!!! GYRO SETTING FAILED! !!!!!!!!!!!!");
        }

        m_gyro.clearStickyFaults();
        m_gyro.setYaw(0);
        m_log_yaw = child.rotation2dLogger(Level.TRACE, "Yaw NWU (rad)");
        m_log_yaw_rate = child.doubleLogger(Level.TRACE, "Yaw Rate NWU (rad_s)");
        m_log_pitch = child.rotation2dLogger(Level.TRACE, "Pitch NWU (rad)");
        m_log_roll = child.rotation2dLogger(Level.TRACE, "Roll NWU (rad)");

    }

    /** This is latency-compensated to the current Takt time. */
    @Override
    public Rotation2d getYawNWU() {
        QuaternionFrame q = m_gyro.getAngularPositionFrame();
        double t = q.getTimestamp();
        double yaw = q.getYaw();
        double rate = m_gyro.getAngularVelocityYaw();
        double dt = Takt.get() - t;
        // it's ok if takt is slightly behind the gyro, in case a CAN packet came in
        // before we got here.
        if (dt < -0.1) {
            Util.warn("Gyro data is from the distant future!");
            dt = 0;
        }
        if (dt > 0.1) {
            Util.warn("Gyro data is very old!");
            dt = 0;
        }
        double correctedYaw = yaw + rate * dt;
        Rotation2d yawNWU = Rotation2d.fromRotations(correctedYaw);
        m_log_yaw.log(() -> yawNWU);
        return yawNWU;
    }

    @Override
    public double getYawRateNWU() {
        double yawRateRad_S = Units.rotationsToRadians(m_gyro.getAngularVelocityYaw());
        m_log_yaw_rate.log(() -> yawRateRad_S);
        return yawRateRad_S;
    }

    /** Not latency-compensated. */
    @Override
    public Rotation2d getPitchNWU() {
        Rotation2d pitchNWU = Rotation2d.fromRotations(m_gyro.getPitch());
        m_log_pitch.log(() -> pitchNWU);
        return pitchNWU;
    }

    /** Not latency-compensated. */
    @Override
    public Rotation2d getRollNWU() {
        Rotation2d rollNWU = Rotation2d.fromRotations(m_gyro.getRoll());
        m_log_roll.log(() -> rollNWU);
        return rollNWU;
    }

    @Override
    public void periodic() {
        if (m_gyro.isCalibrating())
            Util.println("Redux Gyro Calibrating ......");
        CanandgyroFaults activeFaults = m_gyro.getActiveFaults();
        if (activeFaults.faultsValid())
            Util.warn("Redux Gyro fault!");

    }
}