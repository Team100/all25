package org.team100.lib.spline;

import java.util.Optional;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Holonomic spline.
 * 
 * Internally this is three one-dimensional splines (x, y, heading), with
 * respect to a parameter [0,1].
 * 
 * If you don't care about rotation, just pass zero.
 * 
 * Note that some nonholonomic spline consumers assume that dx carries all the
 * motion; that's not true here.
 * 
 * This happily produces "splines" with sharp corners, if the segment
 * derivatives don't match. To fix that, use SplineUtil.forceC1().
 */
public class HolonomicSpline {
    private static final boolean DEBUG = false;
    private static final int kSamples = 100;

    private final Spline1d m_x;
    private final Spline1d m_y;
    private final Spline1d m_theta;
    /**
     * Offset for rotational spline: the rotational spline doesn't include the
     * starting point in order to correctly handle wrapping.
     */
    private final Rotation2d m_r0;

    /**
     * The theta endpoint derivative is just the average theta rate, which is new,
     * it used to be zero.
     * 
     * You'll probably want to call SplineUtil.forceC1() and
     * SplineUtil.optimizeSpline() after creating these segments.
     */
    public HolonomicSpline(HolonomicPose2d p0, HolonomicPose2d p1) {
        this(p0, p1, 1.2, 1.2);
    }

    /**
     * Specify the magic number you want: this scales the derivatives at the
     * endpoints, i.e. how "strongly" the derivative affects the curve. High magic
     * number means low curvature at the endpoint.
     * 
     * The theta endpoint derivative is just the average theta rate, which is new,
     * it used to be zero.
     * 
     * You'll probably want to call SplineUtil.forceC1() and
     * SplineUtil.optimizeSpline() after creating these segments.
     */
    public HolonomicSpline(HolonomicPose2d p0, HolonomicPose2d p1, double mN0, double mN1) {
        double scale0 = mN0 * GeometryUtil.distance(p0.translation(), p1.translation());
        double scale1 = mN1 * GeometryUtil.distance(p0.translation(), p1.translation());
        if (DEBUG)
            Util.printf("scale %f %f\n", scale0, scale1);
        double x0 = p0.translation().getX();
        double x1 = p1.translation().getX();
        // first derivatives are just the course
        double dx0 = p0.course().getCos() * scale0;
        double dx1 = p1.course().getCos() * scale1;
        // second derivatives are zero at the ends
        double ddx0 = 0;
        double ddx1 = 0;

        double y0 = p0.translation().getY();
        double y1 = p1.translation().getY();
        // first derivatives are just the course
        double dy0 = p0.course().getSin() * scale0;
        double dy1 = p1.course().getSin() * scale1;
        // second derivatives are zero at the ends
        double ddy0 = 0;
        double ddy1 = 0;

        m_x = Spline1d.newSpline1d(x0, x1, dx0, dx1, ddx0, ddx1);
        m_y = Spline1d.newSpline1d(y0, y1, dy0, dy1, ddy0, ddy1);

        m_r0 = p0.heading();
        double delta = p0.heading().unaryMinus().rotateBy(p1.heading()).getRadians();

        // previously dtheta at the endpoints was zero, which is bad: it meant the omega
        // value varied all over the place, even for theta paths that should be
        // completely smooth.
        // a reasonable derivative for theta is just the average (i.e. the course from
        // the preceding point to the following point)
        // this will produce a "corner" in theta, which you may want to fix with
        // SplineUtil.forceC1().
        double dtheta0 = delta;
        double dtheta1 = delta;
        // second derivatives are zero at the ends
        double ddtheta0 = 0;
        double ddtheta1 = 0;
        m_theta = Spline1d.newSpline1d(0.0, delta, dtheta0, dtheta1, ddtheta0, ddtheta1);
    }

    @Override
    public String toString() {
        return "HolonomicSpline [m_x=" + m_x + ", m_y=" + m_y + ", m_theta=" + m_theta + ", m_r0=" + m_r0 + "]";
    }

    /** This is used by various optimization steps. */
    private HolonomicSpline(
            Spline1d x,
            Spline1d y,
            Spline1d theta,
            Rotation2d r0) {
        m_x = x;
        m_y = y;
        m_theta = theta;
        m_r0 = r0;
    }

    public Pose2dWithMotion getPose2dWithMotion(double p) {
        Optional<Rotation2d> course = getCourse(p);
        double dx = course.isPresent() ? course.get().getCos() : 0.0;
        double dy = course.isPresent() ? course.get().getSin() : 0.0;
        double dtheta = course.isPresent() ? getDHeadingDs(p) : getDHeading(p);

        return new Pose2dWithMotion(
                getPose2d(p),
                new MotionDirection(dx, dy, dtheta),
                getCurvature(p),
                getDCurvatureDs(p));
    }

    /**
     * Course is the direction of motion, regardless of the direction the robot is
     * facing (heading). It's optional to account for the motionless case.
     *
     * Course is the same for holonomic and nonholonomic splines.
     */
    public Optional<Rotation2d> getCourse(double t) {
        double dx = dx(t);
        double dy = dy(t);
        if (Math100.epsilonEquals(dx, 0.0) && Math100.epsilonEquals(dy, 0.0)) {
            // rotation below would be garbage so give up
            return Optional.empty();
        }
        return Optional.of(new Rotation2d(dx, dy));
    }

    public Pose2d getPose2d(double p) {
        return new Pose2d(getPoint(p), getHeading(p));
    }

    ////////////////////////////////////////////////////////////////////////

    protected Rotation2d getHeading(double t) {
        return m_r0.rotateBy(Rotation2d.fromRadians(m_theta.getPosition(t)));
    }

    protected double getDHeading(double t) {
        return m_theta.getVelocity(t);
    }

    HolonomicSpline replaceFirstDerivatives(
            double dx0,
            double dx1,
            double dy0,
            double dy1,
            double dtheta0,
            double dtheta1) {
        return new HolonomicSpline(
                Spline1d.newSpline1d(
                        m_x.getPosition(0),
                        m_x.getPosition(1),
                        dx0,
                        dx1,
                        m_x.getAcceleration(0),
                        m_x.getAcceleration(1)),
                Spline1d.newSpline1d(
                        m_y.getPosition(0),
                        m_y.getPosition(1),
                        dy0,
                        dy1,
                        m_y.getAcceleration(0),
                        m_y.getAcceleration(1)),
                Spline1d.newSpline1d(
                        m_theta.getPosition(0),
                        m_theta.getPosition(1),
                        dtheta0,
                        dtheta1,
                        m_theta.getAcceleration(0),
                        m_theta.getAcceleration(1)),
                m_r0);
    }

    /**
     * Return a new spline that is a copy of this one, but incrementing the second
     * derivatives by the specified amounts.
     */
    HolonomicSpline addToSecondDerivatives(
            double ddx0_sub,
            double ddx1_sub,
            double ddy0_sub,
            double ddy1_sub) {
        return new HolonomicSpline(
                m_x.addCoefs(Spline1d.newSpline1d(0, 0, 0, 0, ddx0_sub, ddx1_sub)),
                m_y.addCoefs(Spline1d.newSpline1d(0, 0, 0, 0, ddy0_sub, ddy1_sub)),
                m_theta,
                m_r0);
    }

    /**
     * Change in heading per distance traveled, i.e. spatial change in heading.
     * dtheta/ds (radians/meter).
     */
    private double getDHeadingDs(double p) {
        return getDHeading(p) / getVelocity(p);
    }

    /**
     * DCurvatureDs is the change in curvature per distance traveled, i.e. the
     * "spatial change in curvature"
     * 
     * dk/dp / ds/dp = dk/ds
     * rad/mp / m/p = rad/m^2
     */
    private double getDCurvatureDs(double p) {
        return getDCurvature(p) / getVelocity(p);
    }

    /** Returns pose in the nonholonomic sense, where the rotation is the course */
    Optional<Pose2d> getStartPose() {
        double dx = dx(0);
        double dy = dy(0);
        if (Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6) {
            // rotation below would be garbage, so give up.
            return Optional.empty();
        }
        return Optional.of(new Pose2d(
                getPoint(0),
                new Rotation2d(dx, dy)));
    }

    /** Returns pose in the nonholonomic sense, where the rotation is the course */
    Optional<Pose2d> getEndPose() {
        double dx = dx(1);
        double dy = dy(1);
        if (Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6) {
            // rotation below would be garbage, so give up.
            return Optional.empty();
        }
        return Optional.of(new Pose2d(
                getPoint(1),
                new Rotation2d(dx, dy)));
    }

    /**
     * Cartesian coordinate in meters at p.
     * 
     * @param t ranges from 0 to 1
     * @return the point on the spline for that t value
     */
    protected Translation2d getPoint(double t) {
        return new Translation2d(m_x.getPosition(t), m_y.getPosition(t));
    }

    double x(double t) {
        return m_x.getPosition(t);
    }

    double y(double t) {
        return m_y.getPosition(t);
    }

    double theta(double t) {
        return getHeading(t).getRadians();
    }

    double dx(double t) {
        return m_x.getVelocity(t);
    }

    double dy(double t) {
        return m_y.getVelocity(t);
    }

    double dtheta(double t) {
        return m_theta.getVelocity(t);
    }

    double ddx(double t) {
        return m_x.getAcceleration(t);
    }

    double ddy(double t) {
        return m_y.getAcceleration(t);
    }

    double ddtheta(double t) {
        return m_theta.getAcceleration(t);
    }

    double dddx(double t) {
        return m_x.getJerk(t);
    }

    double dddy(double t) {
        return m_y.getJerk(t);
    }

    double dddtheta(double t) {
        return m_theta.getJerk(t);
    }

    /**
     * Velocity is the change in position per parameter, p: ds/dp (meters per p).
     * Since p is not time, it is not "velocity" in the usual sense.
     */
    protected double getVelocity(double t) {
        return Math.hypot(dx(t), dy(t));
    }

    /**
     * Curvature is the change in motion direction per distance traveled.
     * rad/m.
     * Note the denominator is distance in this case, not the parameter, p.
     * but the argument to this function *is* the parameter, p. :-)
     */
    protected double getCurvature(double t) {
        return (dx(t) * ddy(t) - ddx(t) * dy(t))
                / ((dx(t) * dx(t) + dy(t) * dy(t)) * Math.sqrt((dx(t) * dx(t) + dy(t) * dy(t))));
    }

    /**
     * DCurvature is the change in curvature per change in p.
     * dk/dp (rad/m per p)
     * If you want change in curvature per meter, use getDCurvatureDs.
     */
    protected double getDCurvature(double t) {
        double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
        double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2
                - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
        return num / (dx2dy2 * dx2dy2 * Math.sqrt(dx2dy2));
    }

    private double dCurvature2(double t) {
        double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
        if (dx2dy2 == 0)
            throw new IllegalArgumentException();
        double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2
                - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
        return num * num / (dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2);
    }

    /** integrate curvature over the length of the spline. */
    double maxCurvature() {
        double dt = 1.0 / kSamples;
        double maxC = 0;
        for (double t = 0; t < 1.0; t += dt) {
            maxC = Math.max(maxC, getCurvature(t));
        }
        return maxC;
    }

    /** integrate curvature over the length of the spline. */
    double sumCurvature() {
        double dt = 1.0 / kSamples;
        double sum = 0;
        for (double t = 0; t < 1.0; t += dt) {
            sum += (dt * getCurvature(t));
        }
        return sum;
    }

    /**
     * @return integral of dCurvature^2 over the length of the spline
     */
    double sumDCurvature2() {
        double dt = 1.0 / kSamples;
        double sum = 0;
        for (double t = 0; t < 1.0; t += dt) {
            sum += (dt * dCurvature2(t));
        }
        return sum;
    }

}