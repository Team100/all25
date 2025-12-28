package org.team100.lib.state;

import org.team100.lib.geometry.AccelerationSE2;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.timing.TimedState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Describes the state of rigid body transformations in two dimensions, the
 * SE(2) manifold (x,y,theta), where each dimension is represented by position,
 * velocity, and acceleration.
 * 
 * This could be used for navigation, or for other applications of rigid-body
 * transforms in 2d, e.g. planar mechanisms.
 * 
 * This type is used for control, which is why it includes acceleration
 * 
 * Do not try to use zero as an initial location; always initialize with the
 * current location.
 * 
 * Note: the metric used here is not the SE(2) geodesic, it treats the XY plane
 * and rotation dimensions independently.
 */
public class ControlSE2 {
    private final Control100 m_x;
    private final Control100 m_y;
    private final Control100 m_theta;

    public ControlSE2(Control100 x, Control100 y, Control100 theta) {
        m_x = x;
        m_y = y;
        m_theta = theta;
    }

    public ControlSE2(Pose2d x, VelocitySE2 v) {
        this(
                new Control100(x.getX(), v.x(), 0),
                new Control100(x.getY(), v.y(), 0),
                new Control100(x.getRotation().getRadians(), v.theta(), 0));
    }

    public ControlSE2(Pose2d x, VelocitySE2 v, AccelerationSE2 a) {
        this(
                new Control100(x.getX(), v.x(), a.x()),
                new Control100(x.getY(), v.y(), a.y()),
                new Control100(x.getRotation().getRadians(), v.theta(), a.theta()));
    }

    public ControlSE2(Pose2d x) {
        this(x, new VelocitySE2(0, 0, 0));
    }

    public ControlSE2(Rotation2d x) {
        this(new Pose2d(0, 0, x));
    }

    public static ControlSE2 zero() {
        return new ControlSE2(new Control100(), new Control100(), new Control100());
    }

    public ModelSE2 model() {
        return new ModelSE2(m_x.model(), m_y.model(), m_theta.model());
    }

    /** Component-wise difference (not geodesic) */
    public ControlSE2 minus(ControlSE2 other) {
        return new ControlSE2(x().minus(other.x()), y().minus(other.y()), theta().minus(other.theta()));
    }

    /** Component-wise sum (not geodesic) */
    public ControlSE2 plus(ControlSE2 other) {
        return new ControlSE2(x().plus(other.x()), y().plus(other.y()), theta().plus(other.theta()));
    }

    public boolean near(ControlSE2 other, double tolerance) {
        return x().near(other.x(), tolerance)
                && y().near(other.y(), tolerance)
                && theta().near(other.theta(), tolerance);
    }

    public Pose2d pose() {
        return new Pose2d(m_x.x(), m_y.x(), rotation());
    }

    /** Translation of the pose */
    public Translation2d translation() {
        return new Translation2d(m_x.x(), m_y.x());
    }

    public Rotation2d rotation() {
        return new Rotation2d(m_theta.x());
    }

    public VelocitySE2 velocity() {
        return new VelocitySE2(m_x.v(), m_y.v(), m_theta.v());
    }

    /** Robot-relative speeds */
    public ChassisSpeeds chassisSpeeds() {
        return SwerveKinodynamics.toInstantaneousChassisSpeeds(velocity(), rotation());
    }

    public AccelerationSE2 acceleration() {
        return new AccelerationSE2(m_x.a(), m_y.a(), m_theta.a());
    }

    public Control100 x() {
        return m_x;
    }

    public Control100 y() {
        return m_y;
    }

    public Control100 theta() {
        return m_theta;
    }

    /**
     * Transform timed pose into swerve state.
     * 
     * Correctly accounts for centripetal acceleration.
     */
    public static ControlSE2 fromTimedState(TimedState timedPose) {
        double xx = timedPose.state().waypoint().pose().getTranslation().getX();
        double yx = timedPose.state().waypoint().pose().getTranslation().getY();
        double thetax = timedPose.state().waypoint().pose().getRotation().getRadians();

        double velocityM_s = timedPose.velocityM_S();
        Rotation2d course = timedPose.state().waypoint().course().toRotation();
        double xv = course.getCos() * velocityM_s;
        double yv = course.getSin() * velocityM_s;
        double thetav = timedPose.state().getHeadingRateRad_M() * velocityM_s;

        double accelM_s_s = timedPose.acceleration();
        double xa = course.getCos() * accelM_s_s;
        double ya = course.getSin() * accelM_s_s;
        double thetaa = timedPose.state().getHeadingRateRad_M() * accelM_s_s;

        // centripetal accel = v^2/r = v^2 * curvature
        double curvRad_M = timedPose.state().getCurvatureRad_M();
        double centripetalAccelM_s_s = velocityM_s * velocityM_s * curvRad_M;
        double xCa = -1.0 * course.getSin() * centripetalAccelM_s_s;
        double yCa = course.getCos() * centripetalAccelM_s_s;

        return new ControlSE2(
                new Control100(xx, xv, xa + xCa),
                new Control100(yx, yv, ya + yCa),
                new Control100(thetax, thetav, thetaa));
    }

    public String toString() {
        return "SwerveControl(" + m_x + ", " + m_y + ", " + m_theta + ")";
    }

}