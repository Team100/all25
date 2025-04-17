package org.team100.lib.motion.drivetrain.module;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePosition100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Feedforward and feedback control of a single module.
 * 
 * It used to be that the turning servo would contain the profile, but now it's
 * here.
 */
public abstract class SwerveModule100 implements Glassy {
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
        // note this will be wrong at instantiation.
        // TODO: make previous desired angle null instead.
        double measurement = m_turningServo.getPosition().orElse(0);
        // the default previous angle is the measurement.
        m_previousDesiredAngle = new Rotation2d(measurement);
        m_previousTime = Takt.get();
    }

    /**
     * Optimizes.
     * 
     * Works fine with empty angles.
     * 
     * TODO: make it do nothing if the angle is empty
     */
    void setDesiredState(SwerveModuleState100 desired) {
        desired = usePreviousAngleIfEmpty(desired);
        desired = optimize(desired);
        actuate(desired);
    }

    /**
     * Does not optimize.
     * 
     * Works fine with empty angles.
     * 
     * TODO: make it do nothing if the angle is empty
     */
    void setRawDesiredState(SwerveModuleState100 desired) {
        desired = usePreviousAngleIfEmpty(desired);
        actuate(desired);
    }

    /** Turning servo commands always specify zero-velocity goal. */
    private void actuate(SwerveModuleState100 desired) {
        double speed = desired.speedMetersPerSecond();
        if (desired.angle().isEmpty())
            throw new IllegalArgumentException("actuation needs a real angle");
        Rotation2d desiredAngle = desired.angle().get();

        if (Experiments.instance.enabled(Experiment.CorrectSpeedForSteering)) {
            // help drive motors overcome steering.
            speed = correctSpeedForSteering(speed, desiredAngle);
        }
        if (Experiments.instance.enabled(Experiment.ReduceCrossTrackError)) {
            OptionalDouble optMeasurement = m_turningServo.getPosition();
            if (optMeasurement.isPresent()) {
                double measuredAngleRad = optMeasurement.getAsDouble();
                speed = reduceCrossTrackError(measuredAngleRad, speed, desiredAngle);
            }
        }
        m_driveServo.setVelocityM_S(speed);
        if (Experiments.instance.enabled(Experiment.UnprofiledSteering)) {
            // no profile, just low-level position
            Control100 control = new Control100(desiredAngle.getRadians(), 0);
            m_turningServo.setPositionDirect(new Setpoints1d(control, control), 0);
        } else {
            // use the profile
            m_turningServo.setPositionProfiled(desiredAngle.getRadians(), 0);
        }
        m_previousDesiredAngle = desiredAngle;
    }

    static double reduceCrossTrackError(double measuredAngleRad, double desiredSpeed, Rotation2d desiredAngle) {
        double error = MathUtil.angleModulus(desiredAngle.getRadians() - measuredAngleRad);
        // cosine is pretty forgiving of misalignment
        // double scale = Math.abs(Math.cos(error));
        // gaussian is much less forgiving. note the adjustable factor. The value of
        // 4 means there is almost no motion past about 60 degrees of error.
        final double width = 4.0;
        double scale = Math.exp(-width * error * error);
        return scale * desiredSpeed;
    }

    /** Correct the desired speed for steering coupling. */
    private double correctSpeedForSteering(double desiredSpeed, Rotation2d desiredAngle) {
        Rotation2d dtheta = desiredAngle.minus(m_previousDesiredAngle);
        double now = Takt.get();
        double dt = now - m_previousTime;
        if (dt > 1e-6) {
            // avoid short intervals
            double omega = dtheta.getRadians() / dt;
            // TODO: should this be positive or negative?
            desiredSpeed += .0975 * (omega) / 3.8;
        }
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

    /** Make sure the setpoint and measurement are the same. */
    public void reset() {
        m_turningServo.reset();
        m_driveServo.reset();
    }

    public void close() {
        m_turningServo.close();
    }

    /////////////////////////////////////////////////////////////
    //
    // Package private for SwerveModuleCollection
    //

    /** FOR TEST ONLY */
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
                // TODO: replace the magic number here with .. i think this is like wheel
                // diameter? radius? what is this?
                drive_M -= 0.0975 * (steerRad) / 3.8;
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

    boolean atSetpoint() {
        return m_turningServo.atSetpoint();
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
