package org.team100.lib.motion.drivetrain;

import java.util.Optional;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.state.Model100;
import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Describes the state of a holonomic drive in three dimensions,
 * x, y, and theta, each of which is represented by position and velocity.
 * 
 * This type is used for measurement and estimation, which is why it doesn't
 * include acceleration.
 */
public class SwerveModel {
    private final Model100 m_x;
    private final Model100 m_y;
    private final Model100 m_theta;

    public SwerveModel(Model100 x, Model100 y, Model100 theta) {
        m_x = x;
        m_y = y;
        m_theta = theta;
    }

    public SwerveModel(Pose2d x, FieldRelativeVelocity v) {
        this(
                new Model100(x.getX(), v.x()),
                new Model100(x.getY(), v.y()),
                new Model100(x.getRotation().getRadians(), v.theta()));
    }

    /** Motionless at the origin with the specified pose */
    public SwerveModel(Pose2d x) {
        this(x, new FieldRelativeVelocity(0, 0, 0));
    }

    /** Motionless at the origin with the specified heading */
    public SwerveModel(Rotation2d x) {
        this(new Pose2d(0, 0, x));
    }

    /** Motionless at the origin */
    public SwerveModel() {
        this(new Model100(), new Model100(), new Model100());
    }

    public SwerveControl control() {
        return new SwerveControl(m_x.control(), m_y.control(), m_theta.control());
    }

    public SwerveModel withTheta(double theta) {
        return new SwerveModel(m_x, m_y, new Model100(theta, m_theta.v()));
    }

    public SwerveModel minus(SwerveModel other) {
        return new SwerveModel(x().minus(other.x()), y().minus(other.y()), theta().minus(other.theta()));
    }

    public SwerveModel plus(SwerveModel other) {
        return new SwerveModel(x().plus(other.x()), y().plus(other.y()), theta().plus(other.theta()));
    }

    public boolean near(SwerveModel other, double tolerance) {
        return x().near(other.x(), tolerance)
                && y().near(other.y(), tolerance)
                && theta().near(other.theta(), tolerance);
    }

    public Pose2d pose() {
        return new Pose2d(m_x.x(), m_y.x(), new Rotation2d(m_theta.x()));
    }

    /** Translation of the pose */
    public Translation2d translation() {
        return new Translation2d(m_x.x(), m_y.x());
    }

    public Rotation2d rotation() {
        return new Rotation2d(m_theta.x());
    }

    public FieldRelativeVelocity velocity() {
        return new FieldRelativeVelocity(m_x.v(), m_y.v(), m_theta.v());
    }

    /** Robot-relative speeds */
    public ChassisSpeeds chassisSpeeds() {
        return SwerveKinodynamics.toInstantaneousChassisSpeeds(velocity(), rotation());
    }

    public Model100 x() {
        return m_x;
    }

    public Model100 y() {
        return m_y;
    }

    public Model100 theta() {
        return m_theta;
    }

    /**
     * Transform timed pose into swerve state.
     */
    public static SwerveModel fromTimedPose(TimedPose timedPose) {
        double xx = timedPose.state().getPose().getX();
        double yx = timedPose.state().getPose().getY();
        double thetax = timedPose.state().getHeading().getRadians();

        Optional<Rotation2d> course = timedPose.state().getCourse();
        if (course.isPresent()) {
            Rotation2d motion_direction = course.get();
            double velocityM_s = timedPose.velocityM_S();
            double xv = motion_direction.getCos() * velocityM_s;
            double yv = motion_direction.getSin() * velocityM_s;
            double thetav = timedPose.state().getHeadingRate() * velocityM_s;
            return new SwerveModel(
                    new Model100(xx, xv),
                    new Model100(yx, yv),
                    new Model100(thetax, thetav));
        }

        // no course means no velocity.
        // this is one of the reasons that pure rotations don't work.
        return new SwerveModel(
                new Model100(xx, 0),
                new Model100(yx, 0),
                new Model100(thetax, 0));
    }

    public String toString() {
        return "SwerveModel(" + m_x + ", " + m_y + ", " + m_theta + ")";
    }

}