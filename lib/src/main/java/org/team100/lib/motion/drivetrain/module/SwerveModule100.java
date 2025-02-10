package org.team100.lib.motion.drivetrain.module;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePosition100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.state.Control100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Feedforward and feedback control of a single module.
 */
public abstract class SwerveModule100 implements Glassy {
    private final LinearVelocityServo m_driveServo;
    private final AngularPositionServo m_turningServo;
    private Rotation2d m_previousPosition = new Rotation2d();

    protected SwerveModule100(
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        m_driveServo = driveServo;
        m_turningServo = turningServo;
    }

    /**
     * Optimizes.
     * 
     * Works fine with empty angles.
     */
    void setDesiredState(SwerveModuleState100 desiredState) {
        OptionalDouble position = m_turningServo.getPosition();

        if (position.isPresent()) {
            // Use this in case we get an empty angle.
            m_previousPosition = new Rotation2d(position.getAsDouble());
        } else {
            // This should never happen.
            Util.warn("Empty steering angle measurement!");
        }

        if (desiredState.angle().isEmpty()) {
            desiredState = new SwerveModuleState100(
                    desiredState.speedMetersPerSecond(), Optional.of(m_previousPosition));
        }

        Rotation2d currentAngle = new Rotation2d(position.getAsDouble());
        SwerveModuleState100 optimized = SwerveModuleState100.optimize(
                desiredState, currentAngle);
        // Util.printf("optimized %s\n", optimized);
        setRawDesiredState(optimized);
    }

    /**
     * Does not optimize.
     * 
     * Works fine with empty angles.
     * 
     * Turning servo commands always include zero velocity.
     */
    void setRawDesiredState(SwerveModuleState100 desiredState) {
        if (desiredState.angle().isEmpty()) {
            desiredState = new SwerveModuleState100(
                    desiredState.speedMetersPerSecond(), Optional.of(m_previousPosition));
        }
        m_driveServo.setVelocityM_S(desiredState.speedMetersPerSecond());
        // Util.printf("desired angle %f\n", desiredState.angle().get().getRadians());
        m_turningServo.setPosition(desiredState.angle().get().getRadians(), 0);
    }

    /**
     * The state corresponding to the current setpoint (for drive) and goal (for
     * angle).
     */
    SwerveModuleState100 getDesiredState() {
        return new SwerveModuleState100(
                m_driveServo.getSetpoint(),
                Optional.of(new Rotation2d(m_turningServo.getGoal())));
    }

    /** Make sure the setpoint and measurement are the same. */
    public void reset() {
        m_turningServo.reset();
        m_driveServo.reset();
    }

    /** for testing only */
    Control100 getSetpoint() {
        return m_turningServo.getSetpoint();
    }

    public void close() {
        m_turningServo.close();
    }

    /////////////////////////////////////////////////////////////
    //
    // Package private for SwerveModuleCollection
    //

    /** @return current measurements */
    public SwerveModuleState100 getState() {
        OptionalDouble driveVelocity = m_driveServo.getVelocity();
        OptionalDouble turningPosition = m_turningServo.getPosition();
        if (driveVelocity.isEmpty()) {
            Util.warn("no drive velocity measurement!");
            return null;
        }
        if (turningPosition.isEmpty()) {
            Util.warn("no turning position measurement!");
            return null;
        }
        return new SwerveModuleState100(
                driveVelocity.getAsDouble(),
                Optional.of(new Rotation2d(turningPosition.getAsDouble())));
    }

    public SwerveModulePosition100 getPosition() {
        OptionalDouble driveDistance = m_driveServo.getDistance();
        OptionalDouble turningPosition = m_turningServo.getPosition();
        if (driveDistance.isEmpty()) {
            Util.warn("no drive distance measurement!");
            return null;
        }
        if (turningPosition.isEmpty()) {
            Util.warn("no turning position measurement!");
            return null;
        }
        return new SwerveModulePosition100(
                driveDistance.getAsDouble(),
                Optional.of(new Rotation2d(turningPosition.getAsDouble())));
    }

    public OptionalDouble turningPosition() {
        OptionalDouble position = m_turningServo.getPosition();
        Util.printf("position %s\n", position);
        return position;
    }

    public OptionalDouble turningVelocity() {
        return m_turningServo.getVelocity();
    }

    boolean atSetpoint() {
        return m_turningServo.atSetpoint();
    }

    boolean atGoal() {
        boolean atGoal = m_turningServo.atGoal();
        Util.printf("module atgoal %b\n", atGoal);
        return atGoal;
    }

    void stop() {
        m_driveServo.stop();
        m_turningServo.stop();
    }

    /** Update logs. */
    void periodic() {
        m_driveServo.periodic();
        m_turningServo.periodic();
    }
}
