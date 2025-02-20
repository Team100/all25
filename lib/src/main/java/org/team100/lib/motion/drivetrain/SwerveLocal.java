package org.team100.lib.motion.drivetrain;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.logging.LoggerFactory.SwerveModulePosition100Logger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePositions;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.state.Control100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The swerve drive in local, or robot, reference frame. This class knows
 * nothing about the outside world, it just accepts chassis speeds.
 */
public class SwerveLocal implements Glassy, SwerveLocalObserver {
    private static final double kPositionToleranceRad = 0.05; // about 3 degrees
    private static final double kVelocityToleranceRad_S = 0.05; // 3 deg/s, slow!
    private static final SwerveModuleStates states0 = new SwerveModuleStates(
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
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
        m_log_chassis_speed = child.chassisSpeedsLogger(Level.TRACE, "chassis speed LOG");
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
     * Discretizes the speeds, calculates the inverse kinematic module states,
     * and aligns the wheels with the directions implied by speeds,
     * without moving the drive motors.
     */
    public void steerAtRest(ChassisSpeeds speeds) {
        SwerveModuleStates states = m_swerveKinodynamics.toSwerveModuleStates(speeds);
        states = states.motionless();
        setModuleStates(states);
    }

    /**
     * True if current wheel steering positions are aligned with the positions
     * implied by the desired speed.
     */
    public boolean aligned(ChassisSpeeds desiredSpeeds) {
        // These measurements include empty angles for motionless wheels.
        // Otherwise angle is always within [-pi, pi].
        OptionalDouble[] positions = m_modules.turningPosition();
        OptionalDouble[] velocities = m_modules.turningVelocity();
        SwerveModuleState100[] desiredStates = m_swerveKinodynamics.toSwerveModuleStates(
                desiredSpeeds).all();
        for (int i = 0; i < positions.length; ++i) {
            if (positions[i].isEmpty() && velocities[i].isEmpty()) {
                // Util.warn("broken sensor, this should never happen");
                return false;
            }
            double position = positions[i].getAsDouble();
            double velocity = velocities[i].getAsDouble();
            if (desiredStates[i].angle().isEmpty()) {
                // This can happen if the desired wheel speed is zero, which can happen even if
                // the other desired wheel speeds are not zero.
                continue;
            }
            double setpoint = desiredStates[i].angle().get().getRadians();
            // Modulus accommodates "optimizing" wheel direction.
            double error = MathUtil.inputModulus(setpoint - position, -Math.PI / 2, Math.PI / 2);
            if (Math.abs(error) > kPositionToleranceRad) {
                return false;
            }
            double velocityError = velocity;
            // steering commands always specify zero speed.
            if (Math.abs(velocityError) > kVelocityToleranceRad_S) {
                return false;
            }
        }
        return true;
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
    public SwerveModuleStates getDesiredStates() {
        return m_modules.getDesiredStates();
    }

    public Control100[] getSetpoints() {
        return m_modules.getSetpoint();
    }

    @Override
    public SwerveModuleStates states() {
        return m_modules.states();
    }

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

    @Override
    public boolean[] atGoal() {
        return m_modules.atGoal();
    }

    ///////////////////////////////////////////

    public void close() {
        m_modules.close();
    }

    public void reset() {
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
