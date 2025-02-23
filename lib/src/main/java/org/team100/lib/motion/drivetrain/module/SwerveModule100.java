package org.team100.lib.motion.drivetrain.module;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePosition100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.state.Control100;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Feedforward and feedback control of a single module.
 */
public abstract class SwerveModule100 implements Glassy {
    private static final boolean DEBUG = false;
    private final LinearVelocityServo m_driveServo;
    private final AngularPositionServo m_turningServo;
    /**
     * The previous desired angle, used if the current desired angle is empty (i.e.
     * the module is motionless) and to calculate steering velocity.
     */
    private Rotation2d m_previousDesiredAngle;
    private double m_previousTime;

    protected SwerveModule100(
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        m_driveServo = driveServo;
        m_turningServo = turningServo;
        // the default previous angle is the measurement.
        m_previousDesiredAngle = new Rotation2d(m_turningServo.getPosition().orElse(0));
        m_previousTime = Takt.get();
    }

    /**
     * Optimizes.
     * 
     * Works fine with empty angles.
     */
    void setDesiredState(SwerveModuleState100 desired) {
        desired = usePreviousAngleIfEmpty(desired);
        desired = optimize(desired);
        m_driveServo.setVelocityM_S(correctSpeed(desired));
        m_turningServo.setPosition(desired.angle().get().getRadians(), 0);
    }

    /**
     * Does not optimize.
     * 
     * Works fine with empty angles.
     * 
     * Turning servo commands always include zero velocity.
     */
    void setRawDesiredState(SwerveModuleState100 desired) {
        desired = usePreviousAngleIfEmpty(desired);
        m_driveServo.setVelocityM_S(correctSpeed(desired));
        m_turningServo.setPosition(desired.angle().get().getRadians(), 0);
    }

    /** Correct the desired speed for steering coupling. */
    private double correctSpeed(SwerveModuleState100 desired) {
        Rotation2d desiredAngle = desired.angle().get();
        double desiredSpeed = desired.speedMetersPerSecond();
        Rotation2d dtheta = desiredAngle.minus(m_previousDesiredAngle);
        double now = Takt.get();
        double dt = now - m_previousTime;
        if (dt > 1e-6) {
            // avoid short intervals
            double omega = dtheta.getRadians() / dt;
            // System.out.println(omega);
            // TODO: should this be positive or negative?
            // desiredSpeed += .0975 * (omega) / 3.8;
        }

        m_previousDesiredAngle = desiredAngle;
        m_previousTime = now;
        return desiredSpeed;
    }

    /**
     * Use the current turning servo position to optimize the desired state.
     */
    private SwerveModuleState100 optimize(SwerveModuleState100 desired) {
        OptionalDouble position = m_turningServo.getPosition();
        if (position.isEmpty()) {
            // This should never happen.
            Util.warn("Empty steering angle measurement!");
            return desired;
        }
        return SwerveModuleState100.optimize(
                desired,
                new Rotation2d(position.getAsDouble()));
    }

    /**
     * If the desired angle is empty, replace it with the previous desired angle.
     */
    private SwerveModuleState100 usePreviousAngleIfEmpty(SwerveModuleState100 desired) {
        if (desired.angle().isEmpty()) {
            return new SwerveModuleState100(
                    desired.speedMetersPerSecond(),
                    Optional.of(m_previousDesiredAngle));
        }
        return desired;
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

    /**
     * FOR TESTING ONLY
     * 
     * TODO: remove this
     * 
     * @return current measurements
     */
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
        double drive_M = driveDistance.getAsDouble();
        double steerRad = turningPosition.getAsDouble();
        switch (Identity.instance) {
            case SWERVE_ONE:
            case SWERVE_TWO:
            case COMP_BOT:
                drive_M -= .0975 * (steerRad) / 3.8;
                break;
            case BLANK:
            default:
                break;
        }
        return new SwerveModulePosition100(
                drive_M,
                Optional.of(new Rotation2d(steerRad)));
    }

    public OptionalDouble turningPosition() {
        OptionalDouble position = m_turningServo.getPosition();
        // Util.printf("position %s\n", position);
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
        // Util.printf("module atgoal %b\n", atGoal);
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
