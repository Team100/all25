package org.team100.lib.motion.drivetrain;

import java.util.Optional;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.logging.LoggerFactory.SwerveModulePosition100Logger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePositions;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The swerve drive in local, or robot, reference frame. This class knows
 * nothing about the outside world, it just accepts chassis speeds.
 */
public class SwerveLocal implements Glassy, SwerveLocalObserver {
    private static final boolean DEBUG = false;
    private static final SwerveModuleStates states0 = new SwerveModuleStates(
            new SwerveModuleState100(0, Optional.of(Rotation2d.kZero)),
            new SwerveModuleState100(0, Optional.of(Rotation2d.kZero)),
            new SwerveModuleState100(0, Optional.of(Rotation2d.kZero)),
            new SwerveModuleState100(0, Optional.of(Rotation2d.kZero)));
    private static final SwerveModuleStates states90 = new SwerveModuleStates(
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))));
    private static final SwerveModuleStates statesX = new SwerveModuleStates(
            // note range is [-pi,pi]
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(-1 * Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(3 * Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(-3 * Math.PI / 4))));

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final SwerveModuleCollection m_modules;
    private final SwerveModulePosition100Logger m_swerveModuleFrontLeftPosition;
    private final SwerveModulePosition100Logger m_swerveModuleFrontRightPosition;
    private final SwerveModulePosition100Logger m_swerveModuleRearLeftPosition;
    private final SwerveModulePosition100Logger m_swerveModuleRearRightPosition;
    private final ChassisSpeedsLogger m_log_chassis_speed;

    public SwerveLocal(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            SwerveModuleCollection modules) {
        LoggerFactory child = parent.child(this);
        m_log_chassis_speed = child.chassisSpeedsLogger(Level.TRACE, "chassis speed");
        m_swerveModuleFrontLeftPosition = child.swerveModulePosition100Logger(Level.TRACE, "Fromt Left");
        m_swerveModuleFrontRightPosition = child.swerveModulePosition100Logger(Level.TRACE, "Front Right");
        m_swerveModuleRearLeftPosition = child.swerveModulePosition100Logger(Level.TRACE, "Rear Left");
        m_swerveModuleRearRightPosition = child.swerveModulePosition100Logger(Level.TRACE, "Rear Right");
        m_swerveKinodynamics = swerveKinodynamics;
        m_modules = modules;
    }

    //////////////////////////////////////////////////////////
    //
    // Actuators. These are mutually exclusive within an iteration.
    //

    /**
     * Discretizes the speeds, calculates the inverse kinematic module states, and
     * sets the module states.
     */
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleStates states = m_swerveKinodynamics.toSwerveModuleStates(speeds);
        setModuleStates(states);
        m_log_chassis_speed.log(() -> speeds);
    }

    /**
     * Sets the wheels to make an "X" pattern.
     */
    public void defense() {
        // not optimizing makes it easier to test, not sure it's worth the slowness.
        setRawModuleStates(statesX);
    }

    /**
     * Sets wheel rotation to zero, for optimizing steering control.
     */
    public void steer0() {
        setRawModuleStates(states0);
    }

    /**
     * Sets wheel rotation to 90 degrees, for optimizing steering control.
     */
    public void steer90() {
        setRawModuleStates(states90);
    }

    public void stop() {
        m_modules.stop();
    }

    /**
     * Set the module states without desaturating.
     * 
     * Works fine with empty angles.
     * 
     * This "raw" mode is just for testing.
     */
    public void setRawModuleStates(SwerveModuleStates targetModuleStates) {
        m_modules.setRawDesiredStates(targetModuleStates);
    }

    ////////////////////////////////////////////////////////////////////
    //
    // Observers
    //

    @Override
    public SwerveModulePositions positions() {
        return m_modules.positions();
    }

    public Translation2d[] getModuleLocations() {
        return m_swerveKinodynamics.getKinematics().getModuleLocations();
    }

    public boolean[] atSetpoint() {
        return m_modules.atSetpoint();
    }

    ///////////////////////////////////////////

    public void close() {
        m_modules.close();
    }

    public void reset() {
        if (DEBUG)
            Util.warn("make sure resetting in SwerveLocal doesn't break anything");
        m_modules.reset();
    }

    /** Updates visualization. */
    void periodic() {
        m_swerveModuleFrontLeftPosition.log(() -> positions().frontLeft());
        m_swerveModuleFrontRightPosition.log(() -> positions().frontRight());
        m_swerveModuleRearLeftPosition.log(() -> positions().rearLeft());
        m_swerveModuleRearRightPosition.log(() -> positions().rearRight());
        m_modules.periodic();
    }

    /////////////////////////////////////////////////////////

    private void setModuleStates(SwerveModuleStates states) {
        m_modules.setDesiredStates(states);
    }
}
