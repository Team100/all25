package org.team100.lib.motion.drivetrain;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

public class MockDrive implements DriveSubsystemInterface {
    public boolean m_aligned;
    public FieldRelativeVelocity m_setpoint;
    public FieldRelativeVelocity m_atRestSetpoint;
    // for when you don't care how it was set
    public FieldRelativeVelocity m_recentSetpoint;
    public SwerveModel m_state;

    @Override
    public void driveInFieldCoords(FieldRelativeVelocity setpoint) {
        m_setpoint = setpoint;
        m_recentSetpoint = setpoint;
    }

    @Override
    public void steerAtRest(FieldRelativeVelocity setpoint) {
        m_atRestSetpoint = setpoint;
        m_recentSetpoint = setpoint;
    }

    @Override
    public boolean aligned(FieldRelativeVelocity v) {
        return m_aligned;
    }

    @Override
    public SwerveModel getState() {
        return m_state;
    }

    @Override
    public void stop() {
        // do nothing
    }

    @Override
    public void driveInFieldCoordsVerbatim(FieldRelativeVelocity setpoint) {
        driveInFieldCoords(setpoint);
    }

    @Override
    public void resetLimiter() {
        //
    }

}
