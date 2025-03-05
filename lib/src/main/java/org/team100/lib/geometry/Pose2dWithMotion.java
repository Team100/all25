package org.team100.lib.geometry;

import java.util.Optional;

import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents a state within a 2d holonomic path, i.e. with heading independent
 * from course.
 * 
 * This is a purely spatial construct: for the notion of velocity, you probably
 * want TimedPose.
 */
public class Pose2dWithMotion {
    /**
     * Pose2dWithMotion is purely a spatial construct. It has no sense of time. So
     * rather than being in units of distance-per-time (or radians-per-time), the
     * denominator here is actually distance as well. Thus, it isn't really
     * meaningful to look at dx or dy directly - what is meaningful is:
     *
     * a) whether or not they are both zero (in which case this is a stationary or
     * turn-in-place motion)
     *
     * b) the angle formed by them, which is the direction of translation (in the
     * same coordinate frame as pose).
     *
     * Additionally, this means dtheta is in radians-per-distance if there is
     * translation, or radians-per-radian otherwise.
     */
    public static class MotionDirection {
        private final double m_dx;
        private final double m_dy;
        private final double m_dtheta;

        public MotionDirection(double dx, double dy, double dtheta) {
            m_dx = dx;
            m_dy = dy;
            m_dtheta = dtheta;
        }

        /**
         * Magnitude of the translational part of the motion.
         */
        double norm() {
            return Math.hypot(m_dx, m_dy);
        }

        /**
         * Direction of the translational part of the motion, or empty if motionless.
         */
        Optional<Rotation2d> course() {
            if (norm() > 1e-12)
                return Optional.of(new Rotation2d(m_dx, m_dy));
            return Optional.empty();
        }

        MotionDirection interpolate(MotionDirection other, double x) {
            return new MotionDirection(
                    MathUtil.interpolate(m_dx, other.m_dx, x),
                    MathUtil.interpolate(m_dy, other.m_dy, x),
                    MathUtil.interpolate(m_dtheta, other.m_dtheta, x));
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            MotionDirection other = (MotionDirection) obj;
            if (!MathUtil.isNear(m_dx, other.m_dx, 1e-6))
                return false;
            if (!MathUtil.isNear(m_dy, other.m_dy, 1e-6))
                return false;
            if (!MathUtil.isNear(m_dtheta, other.m_dtheta, 1e-6))
                return false;
            return true;
        }

        @Override
        public String toString() {
            Optional<Rotation2d> course = course();
            String courseStr = course.map((x) -> String.format("%5.3f", x.getRadians())).orElse("empty");
            return String.format("norm: %5.3f, course: %s", norm(), courseStr);
        }

    }

    private final Pose2d m_pose;

    private final MotionDirection m_fieldRelativeMotionDirection;

    /**
     * Curvature is change in angle per change in distance, rad/m.
     */
    private final double m_curvatureRad_M;

    /**
     * DCurvatureDs is change in curvature per meter, rad/m^2
     */
    private final double m_dCurvatureDsRad_M2;

    /**
     * Motionless, no curvature.
     * 
     * @param pose
     */
    public Pose2dWithMotion(Pose2d pose) {
        this(pose, new MotionDirection(0, 0, 0), 0, 0);
    }

    /**
     * 
     * @param pose                         Represents the location and heading of
     *                                     the robot
     * @param fieldRelativeMotionDirection Represents the change in location and
     *                                     heading, per meter traveled.
     * @param curvatureRad_M               Represents the change in course per meter
     *                                     traveled.
     * @param dCurvatureDsRad_M2           Represents the acceleration in course per
     *                                     meter traveled squared.
     */
    public Pose2dWithMotion(
            Pose2d pose,
            MotionDirection fieldRelativeMotionDirection,
            double curvatureRad_M,
            double dCurvatureDsRad_M2) {
        m_pose = pose;
        m_fieldRelativeMotionDirection = fieldRelativeMotionDirection;
        m_curvatureRad_M = curvatureRad_M;
        m_dCurvatureDsRad_M2 = dCurvatureDsRad_M2;
    }

    public final Pose2d getPose() {
        return m_pose;
    }

    /** Radians per meter, which is the reciprocal of the radius. */
    public double getCurvature() {
        return m_curvatureRad_M;
    }

    /** Radians per meter squared */
    public double getDCurvatureDs() {
        return m_dCurvatureDsRad_M2;
    }

    public final Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public final Rotation2d getHeading() {
        return getPose().getRotation();
    }

    // i think the interpolation of motion direction is invalid; it would yield
    // results not on the unit circle, which makes no sense. maybe the consumers of
    // motion direction always derive the angle anyway?
    public Pose2dWithMotion interpolate(final Pose2dWithMotion other, double x) {
        return new Pose2dWithMotion(getPose().interpolate(other.getPose(), x),
                m_fieldRelativeMotionDirection.interpolate(other.m_fieldRelativeMotionDirection, x),
                Math100.interpolate(getCurvature(), other.getCurvature(), x),
                Math100.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

    /**
     * Distance in meters along the arc between the two poses (in either order)
     * produced by a constant twist.
     */
    public double distance(final Pose2dWithMotion other) {
        return GeometryUtil.distance(getPose(), other.getPose());
    }

    public boolean equals(final Object other) {
        if (!(other instanceof Pose2dWithMotion)) {
            return false;
        }

        Pose2dWithMotion p2dwc = (Pose2dWithMotion) other;
        return getPose().equals(p2dwc.getPose()) &&
                m_fieldRelativeMotionDirection.equals(p2dwc.m_fieldRelativeMotionDirection) &&
                Math100.epsilonEquals(getCurvature(), p2dwc.getCurvature()) &&
                Math100.epsilonEquals(getDCurvatureDs(), p2dwc.getDCurvatureDs());
    }

    public String toString() {
        Pose2d pose = getPose();
        return String.format(
                "x %5.3f, y %5.3f, theta %5.3f, direction %s, curvature %5.3f, dcurvature_ds %5.3f",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getRadians(),
                m_fieldRelativeMotionDirection,
                getCurvature(),
                getDCurvatureDs());
    }

    /**
     * Direction of the translational part of the motion, or empty if motionless.
     */
    public Optional<Rotation2d> getCourse() {
        return m_fieldRelativeMotionDirection.course();
    }

    /**
     * Heading rate is radians per meter.
     * 
     * If you want radians per second, multiply by velocity (meters per second).
     */
    public double getHeadingRate() {
        return m_fieldRelativeMotionDirection.m_dtheta;
    }
}