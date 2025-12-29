package org.team100.lib.geometry;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.spline.PoseWithCurvature;

/**
 * Lots of utility functions.
 */
public class GeometryUtil {
    static final boolean DEBUG = false;

    private GeometryUtil() {
    }

    /**
     * Change in course per change in position.
     * 
     * The inverse radius of the circle fit to the three points.
     * 
     * This is to replace the spline-derived curvature.
     * 
     * https://en.wikipedia.org/wiki/Menger_curvature
     * 
     * https://hratliff.com/files/curvature_calculations_and_circle_fitting.pdf
     */
    public static double mengerCurvature(Translation2d t0, Translation2d t1, Translation2d t2) {
        double x1 = t0.getX();
        double x2 = t1.getX();
        double x3 = t2.getX();
        double y1 = t0.getY();
        double y2 = t1.getY();
        double y3 = t2.getY();
        double dx12 = x2 - x1;
        double dx23 = x3 - x2;
        double dx13 = x1 - x3;
        double dy12 = y2 - y1;
        double dy23 = y3 - y2;
        double dy13 = y1 - y3;
        double num = 2 * Math.abs(dx12 * dy23 - dy12 * dx23);
        double den = Math.sqrt((dx12 * dx12 + dy12 * dy12)
                * (dx23 * dx23 + dy23 * dy23)
                * (dx13 * dx13 + dy13 * dy13));
        if (den < 1e-6) {
            // this isn't really zero
            return 0;
        }
        return num / den;
    }

    /** Return a projected onto the direction of b, retaining the omega of a */
    public static ChassisSpeeds project(ChassisSpeeds a, ChassisSpeeds b) {
        double norm = Metrics.translationalNorm(b);
        if (norm < 1e-9) {
            // there's no target course, bail out
            return a;
        }
        double scale = dot(a, b) / (norm * norm);
        if (DEBUG) {
            System.out.printf("project() scale %.8f\n", scale);
        }
        return new ChassisSpeeds(
                b.vxMetersPerSecond * scale,
                b.vyMetersPerSecond * scale,
                a.omegaRadiansPerSecond);
    }

    public static double dot(ChassisSpeeds a, ChassisSpeeds b) {
        return a.vxMetersPerSecond * b.vxMetersPerSecond + a.vyMetersPerSecond * b.vyMetersPerSecond;
    }

    public static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    public static double dot(Translation2d a, VelocitySE2 b) {
        return a.getX() * b.x() + a.getY() * b.y();
    }

    public static Twist2d discretize(ChassisSpeeds continuous, double dt) {
        ChassisSpeeds speeds = ChassisSpeeds.discretize(continuous, dt);
        return new Twist2d(
                speeds.vxMetersPerSecond * dt,
                speeds.vyMetersPerSecond * dt,
                speeds.omegaRadiansPerSecond * dt);
    }

    public static Twist2d scale(Twist2d twist, double scale) {
        return new Twist2d(twist.dx * scale, twist.dy * scale, twist.dtheta * scale);
    }

    public static VelocitySE2 scale(VelocitySE2 v, double scale) {
        return new VelocitySE2(v.x() * scale, v.y() * scale, v.theta() * scale);
    }

    public static Pose2d transformBy(Pose2d a, Pose2d b) {
        return a.transformBy(new Transform2d(b.getTranslation(), b.getRotation()));
    }

    public static Pose2d inverse(Pose2d a) {
        Rotation2d rotation_inverted = a.getRotation().unaryMinus();
        return new Pose2d(a.getTranslation().unaryMinus().rotateBy(rotation_inverted), rotation_inverted);
    }

    public static Twist2d slog(final Pose2d p) {
        return Pose2d.kZero.log(p);
    }

    public static Twist3d slog(final Pose3d p) {
        return Pose3d.kZero.log(p);
    }

    public static Pose2d sexp(final Twist2d delta) {
        return Pose2d.kZero.exp(delta);
    }

    public static Pose2d fromRotation(final Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }

    public static Pose2d fromTranslation(final Translation2d translation) {
        return new Pose2d(translation, Rotation2d.kZero);
    }

    public static Rotation2d fromRadians(double angle_radians) {
        return new Rotation2d(angle_radians);
    }

    public static Rotation2d fromDegrees(double angle_degrees) {
        return fromRadians(Math.toRadians(angle_degrees));
    }

    public static double WrapRadians(double radians) {
        final double twoPi = 2.0 * Math.PI;
        radians = radians % twoPi;
        radians = (radians + twoPi) % twoPi;
        if (radians > Math.PI)
            radians -= twoPi;
        return radians;
    }

    /**
     * Rotations must be identical, translation is allowed along the rotational
     * direction but not any other direction.
     */
    public static boolean isColinear(Pose2d a, final Pose2d other) {
        if (!GeometryUtil.isParallel(a.getRotation(), other.getRotation()))
            return false;
        final Twist2d twist = slog(transformBy(inverse(a), other));
        return (Math.abs(twist.dy - 0.0) <= 1e-12
                && Math.abs(twist.dtheta - 0.0) <= 1e-12);
    }

    // note parallel also means antiparallel.
    public static boolean isParallel(Rotation2d a, Rotation2d b) {
        return Math.abs(a.getRadians() - b.getRadians()) <= 1e-12
                || Math.abs(a.getRadians() - WrapRadians(b.getRadians() + Math.PI)) <= 1e-12;
    }

    public static boolean near(ChassisSpeeds a, ChassisSpeeds b) {
        return MathUtil.isNear(a.vxMetersPerSecond, b.vxMetersPerSecond, 1e-6)
                && MathUtil.isNear(a.vyMetersPerSecond, b.vyMetersPerSecond, 1e-6)
                && MathUtil.isNear(a.omegaRadiansPerSecond, b.omegaRadiansPerSecond, 1e-6);
    }

    public static Rotation2d flip(Rotation2d a) {
        return new Rotation2d(MathUtil.angleModulus(a.getRadians() + Math.PI));
    }

    /** Straight-line (not constant-twist) interpolation. */
    public static Pose2d interpolate(Pose2d a, Pose2d b, double x) {
        if (x <= 0.0) {
            return a;
        } else if (x >= 1.0) {
            return b;
        }
        Translation2d aT = a.getTranslation();
        Translation2d bT = b.getTranslation();
        Rotation2d aR = a.getRotation();
        Rotation2d bR = b.getRotation();
        // each translation axis is interpolated separately
        Translation2d lerpT = aT.interpolate(bT, x);
        Rotation2d lerpR = aR.interpolate(bR, x);
        return new Pose2d(lerpT, lerpR);
    }

    /** Linear interpolation of each component */
    public static DirectionSE2 interpolate(DirectionSE2 a, DirectionSE2 b, double x) {
        return new DirectionSE2(
                MathUtil.interpolate(a.x, b.x, x),
                MathUtil.interpolate(a.y, b.y, x),
                MathUtil.interpolate(a.theta, b.theta, x));
    }

    /** straight-line interpolation of pose, linear interpolation of course */
    public static WaypointSE2 interpolate(
            WaypointSE2 a,
            WaypointSE2 b,
            double x) {
        return new WaypointSE2(
                interpolate(a.pose(), b.pose(), x),
                interpolate(a.course(), b.course(), x),
                MathUtil.interpolate(a.scale(), b.scale(), x));
    }

    /**
     * Interpolate between a and b, treating each dimension separately. This will
     * make straight lines, whereas the WPI Pose3d interpolator will make
     * constant-twist curves.
     */
    public static Pose3d interpolate(Pose3d a, Pose3d b, double x) {
        if (x <= 0.0) {
            return a;
        } else if (x >= 1.0) {
            return b;
        }
        Translation3d aT = a.getTranslation();
        Translation3d bT = b.getTranslation();
        Rotation3d aR = a.getRotation();
        Rotation3d bR = b.getRotation();
        // each translation axis is interpolated separately
        Translation3d lerpT = aT.interpolate(bT, x);
        // Rotation3d lerpR = aR.interpolate(bR, x);
        Rotation3d lerpR = interpolate(aR, bR, x);
        return new Pose3d(lerpT, lerpR);
    }

    /**
     * Interpolate between a and b, treating each dimension separately. This will
     * make straight lines, whereas the WPI Rotation3d interpolator uses quaternion
     * composition.
     */
    public static Rotation3d interpolate(Rotation3d a, Rotation3d b, double x) {
        if (x <= 0.0) {
            return a;
        } else if (x >= 1.0) {
            return b;
        }
        return new Rotation3d(
                MathUtil.interpolate(a.getX(), b.getX(), x),
                MathUtil.interpolate(a.getY(), b.getY(), x),
                MathUtil.interpolate(a.getZ(), b.getZ(), x));
    }

    public static Translation2d inverse(Translation2d a) {
        return new Translation2d(-a.getX(), -a.getY());
    }

    public static Twist2d interpolate(Twist2d a, Twist2d b, double x) {
        return new Twist2d(MathUtil.interpolate(a.dx, b.dx, x),
                MathUtil.interpolate(a.dy, b.dy, x),
                MathUtil.interpolate(a.dtheta, b.dtheta, x));
    }

    public static boolean poseWithCurvatureEquals(PoseWithCurvature a, PoseWithCurvature b) {
        boolean poseEqual = a.poseMeters.equals(b.poseMeters);
        if (!poseEqual) {
            return false;
        }
        return Math.abs(a.curvatureRadPerMeter - b.curvatureRadPerMeter) <= 1e-12;
    }

    /** direction of the translational part of the twist */
    public static Optional<Rotation2d> getCourse(Twist2d t) {
        if (Metrics.translationalNorm(t) > 1e-12) {
            return Optional.of(new Rotation2d(t.dx, t.dy));
        } else {
            return Optional.empty();
        }
    }

    /** robot-relative course */
    public static Optional<Rotation2d> getCourse(ChassisSpeeds t) {
        if (Metrics.translationalNorm(t) > 1e-12) {
            return Optional.of(new Rotation2d(t.vxMetersPerSecond, t.vyMetersPerSecond));
        } else {
            return Optional.empty();
        }
    }

    public static boolean isZero(ChassisSpeeds x) {
        return Math.abs(x.vxMetersPerSecond) < 1E-9
                && Math.abs(x.vyMetersPerSecond) < 1E-9
                && Math.abs(x.omegaRadiansPerSecond) < 1E-9;
    }

    /**
     * Transform the camera-coordinates translation to NWU coordinates. Note an
     * additional transform will be required to account for the camera offset
     * relative to the robot.
     * 
     * @param zForward translation in camera coordinates, z-forward
     * @return translation in WPI coordinates, x-forward.
     */
    public static Translation3d zForwardToXForward(Translation3d zForward) {
        return new Translation3d(zForward.getZ(), -zForward.getX(), -zForward.getY());
    }

    /**
     * Transform the NWU coordinates translation to camera-coordinates.
     * 
     * @param xForward translation in WPI coordinates, x-forward
     * @return translation in camera coordinates, z-forward.
     */
    public static Translation3d xForwardToZForward(Translation3d xForward) {
        return new Translation3d(-xForward.getY(), -xForward.getZ(), xForward.getX());
    }

    /**
     * Transform the NWU coordinates rotation to camera-coordinates.
     * 
     * @param xForward rotation in WPI coordinates, x-forward
     * @return rotation in camera coordinates, z-forward.
     */
    public static Rotation3d xForwardToZForward(Rotation3d xForward) {
        Quaternion q = xForward.getQuaternion();
        Quaternion q2 = new Quaternion(q.getW(), -q.getY(), -q.getZ(), q.getX());
        return new Rotation3d(q2);
    }

    /**
     * Transform the camera-coordinates rotation to NWU coordinates. Note an
     * additional transform will be required to account for the camera orientation
     * relative to the robot.
     * 
     * @param zforward rotation in camera coordinates, z-forward
     * @return rotation in WPI coordinates x-forward
     */
    public static Rotation3d zForwardToXForward(Rotation3d zforward) {
        Quaternion q = zforward.getQuaternion();
        Quaternion q2 = new Quaternion(q.getW(), q.getZ(), -q.getX(), -q.getY());
        return new Rotation3d(q2);
    }

    public static Vector<N3> toVec(Twist2d twist) {
        return VecBuilder.fill(twist.dx, twist.dy, twist.dtheta);
    }

    public static Vector<N6> toVec(Twist3d twist) {
        return VecBuilder.fill(twist.dx, twist.dy, twist.dz, twist.rx, twist.ry, twist.rz);
    }

    public static Vector<N2> toVec(Translation2d t) {
        return VecBuilder.fill(t.getX(), t.getY());
    }

    /** 3-vector for translation */
    public static Vector<N3> toVec3(Translation2d t) {
        return VecBuilder.fill(t.getX(), t.getY(), 0);
    }

    /////////////////////////////////////////////////////////////////
    ///
    /// DANGER ZONE
    ///
    /// Don't use anything below here unless you really know what you're doing
    ///
    /**
     * Change in heading per change in position.
     */
    public static double headingRatio(Pose2d p0, Pose2d p1) {
        Rotation2d h0 = p0.getRotation();
        Rotation2d h1 = p1.getRotation();
        double d = Metrics.doubleGeodesicDistance(p0, p1);
        if (Math.abs(d) < 1e-6)
            return 0;
        return h1.minus(h0).getRadians() / d;
    }
}
