package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;


public class SwerveLimiterTest {
    private static final double kDelta = 0.001;
    private final static double kDt = 0.02; // s
    private final static SwerveKinodynamics kKinematicLimits = SwerveKinodynamicsFactory.limiting();

    /** The setpoint generator never changes the field-relative course. */
    @Test
    void courseInvariant() {
        FieldRelativeVelocity target = new FieldRelativeVelocity(0, 0, 0);
        Rotation2d theta = new Rotation2d();
        SwerveLimiter generator = new SwerveLimiter(kKinematicLimits, () -> 12);

        {// motionless
            FieldRelativeVelocity prevSetpoint = new FieldRelativeVelocity(0, 0, 0);
            FieldRelativeVelocity setpoint = generator.apply(
                    prevSetpoint,
                    target);
            assertTrue(prevSetpoint.angle().isEmpty());
            assertTrue(setpoint.angle().isEmpty());
        }
        {// at max speed, 45 to the left and spinning
            FieldRelativeVelocity speed = new FieldRelativeVelocity(2.640, 2.640, 3.733);
            SwerveModuleStates states = kKinematicLimits.toSwerveModuleStates(speed);
            FieldRelativeVelocity prevSetpoint = speed;
            FieldRelativeVelocity setpoint = generator.apply(
                    prevSetpoint,
                    target);
            assertEquals(Math.PI / 4, prevSetpoint.angle().get().getRadians(), 1e-12);
            assertEquals(3.733, GeometryUtil.norm(prevSetpoint.speeds()), kDelta);
            assertEquals(3.733, prevSetpoint.speeds().omegaRadiansPerSecond, kDelta);
            //
            //
            // ##############
            // ## this is the bug
            // ##############
            //
            //
            assertEquals(0.7853981633974483, setpoint.angle().get().getRadians(), 1e-12);
            assertEquals(3.733, GeometryUtil.norm(setpoint.speeds()), 0.2);
            assertEquals(2.505100964018383, setpoint.speeds().vxMetersPerSecond, 1e-12);
            assertEquals(2.5206083004022424, setpoint.speeds().vyMetersPerSecond, 0.2);
            assertEquals(3.733, setpoint.speeds().omegaRadiansPerSecond, 0.2);
        }
    }

    /** This is pulled from the case below, to isolate the problem. */
    @Test
    void testMakeSpeeds() {
        FieldRelativeVelocity targetSpeed = new FieldRelativeVelocity(2, 0, 3.5);
        double headingRad = 0.005716666136460628;
        FieldRelativeVelocity prevSpeed = new FieldRelativeVelocity(0.16333333, 0, 0.28583333);

        assertEquals(-0.005716666136460628, targetSpeed.angle().get().getRadians(), 1e-12);

        SwerveLimiter generator = new SwerveLimiter(
                kKinematicLimits, () -> 12);
        {
            // min_s == 1, works fine.
            double min_s = 1.0;
            FieldRelativeVelocity newSpeed = generator.makeSpeeds(prevSpeed, targetSpeed, min_s);
            // new course needs to be the same as the target course.
            assertEquals(-0.005716666136460628, newSpeed.angle().get().getRadians(), 1e-12);
        }
        {
            // this is the failure case.
            double min_s = 0.1;
            FieldRelativeVelocity newSpeed = generator.makeSpeeds(prevSpeed, targetSpeed, min_s);
            // new course needs to be the same as the target course.
            // but here we're using the lerp because the projection is too far from the
            // previous
            assertEquals(-0.0032949096257036685, newSpeed.angle().get().getRadians(), 1e-12);
        }
    }

    /** This is pulled from SimulatedDrivingTest, to isolate the problem. */
    @Test
    void courseInvariantRealistic() {
        FieldRelativeVelocity targetSpeed = new FieldRelativeVelocity(2, 0, 3.5);

        // this is the current estimated pose heading. we've been driving for one time
        // step,
        // so we rotated a tiny bit.
        double headingRad = 0.005716666136460628;

        // not going very fast. note the previous instantaneous robot-relative speed has
        // no "y" component at all, because at the previous time, we had heading of zero
        // (and no speed either).
        FieldRelativeVelocity prevSpeed = new FieldRelativeVelocity(0.16333333, 0, 0.28583333);
        SwerveModuleStates states = kKinematicLimits.toSwerveModuleStates(prevSpeed);
        SwerveModel prevSetpoint = new SwerveModel(prevSpeed, states);

        // the previous course is exactly zero: this is the first time step after
        // starting.
        assertEquals(0, GeometryUtil.getCourse(prevSetpoint.speeds()).get().getRadians(), 1e-12);
        assertEquals(0.16333333, GeometryUtil.norm(prevSetpoint.speeds()), 1e-12);
        assertEquals(0.28583333, prevSetpoint.speeds().omegaRadiansPerSecond, 1e-12);

        // field-relative is +x, field-relative course is zero



        // the robot-relative course is the opposite of the heading
        assertEquals(-0.005716666136460628, GeometryUtil.getCourse(targetSpeed).get().getRadians(), 1e-6);
        // the norm is the same as the input
        assertEquals(2, GeometryUtil.norm(targetSpeed), 1e-12);
        assertEquals(1.9999673198172843, targetSpeed.vxMetersPerSecond, 1e-12);
        assertEquals(-0.011433269998955463, targetSpeed.vyMetersPerSecond, 1e-12);
        assertEquals(3.5, targetSpeed.omegaRadiansPerSecond, 1e-12);

        SwerveLimiter generator = new SwerveLimiter(
                kKinematicLimits, () -> 12);
        SwerveSetpoint setpoint = generator.apply(prevSetpoint, targetSpeed);

        // since the real heading of the robot can't change, the course here needs to
        // still be exactly
        // the opposite of the field-relative heading. if not, veering ensues.
        // here we're using the lerp because the projection is too far away,
        // i.e. this is wrong.
        assertEquals(-0.003990219, GeometryUtil.getCourse(setpoint.speeds()).get().getRadians(), 1e-6);
        assertEquals(0.45495839748000433, GeometryUtil.norm(setpoint.speeds()), 1e-12);
        assertEquals(0.79617994009478, setpoint.speeds().omegaRadiansPerSecond, 1e-12);

    }

    @Test
    void motionlessNoOp() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        SwerveLimiter generator = new SwerveLimiter(
                unlimited, () -> 12);

        FieldRelativeVelocity target = new FieldRelativeVelocity(0, 0, 0);
        Rotation2d theta = new Rotation2d();


        SwerveModuleStates targetStates = unlimited.toSwerveModuleStates(target);
        assertEquals(0, target.vxMetersPerSecond, kDelta);
        assertEquals(0, target.vyMetersPerSecond, kDelta);
        assertEquals(0, target.omegaRadiansPerSecond, kDelta);


        SwerveSetpoint prevSetpoint = new SwerveSetpoint();
        SwerveSetpoint setpoint = generator.apply(prevSetpoint, target);
        assertEquals(0, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);


    }

    @Test
    void driveNoOp() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        SwerveLimiter generator = new SwerveLimiter(
                unlimited, () -> 12);

        FieldRelativeVelocity target = new FieldRelativeVelocity(1, 0, 0);
        Rotation2d theta = new Rotation2d();


        assertEquals(1, target.vxMetersPerSecond, kDelta);
        assertEquals(0, target.vyMetersPerSecond, kDelta);
        assertEquals(0, target.omegaRadiansPerSecond, kDelta);

        SwerveSetpoint prevSetpoint = new SwerveSetpoint();
        SwerveSetpoint setpoint = generator.apply(prevSetpoint, target);
        assertEquals(1, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
        assertEquals(1, setpoint.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(1, setpoint.states().frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(1, setpoint.states().rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(1, setpoint.states().rearRight().speedMetersPerSecond(), kDelta);
        assertEquals(0, setpoint.states().frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(0, setpoint.states().frontRight().angle().get().getRadians(), kDelta);
        assertEquals(0, setpoint.states().rearLeft().angle().get().getRadians(), kDelta);
        assertEquals(0, setpoint.states().rearRight().angle().get().getRadians(), kDelta);
    }

    @Test
    void spinNoOp() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        SwerveLimiter generator = new SwerveLimiter(
                unlimited, () -> 12);

        FieldRelativeVelocity target = new FieldRelativeVelocity(0, 0, 1);
        Rotation2d theta = new Rotation2d();


        SwerveModuleStates targetStates = unlimited.toSwerveModuleStates(target);
        assertEquals(0, target.vxMetersPerSecond, kDelta);
        assertEquals(0, target.vyMetersPerSecond, kDelta);
        assertEquals(1, target.omegaRadiansPerSecond, kDelta);
        assertEquals(0.353, targetStates.frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0.353, targetStates.frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(0.353, targetStates.rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0.353, targetStates.rearRight().speedMetersPerSecond(), kDelta);
        assertEquals(2.356, targetStates.frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(0.785, targetStates.frontRight().angle().get().getRadians(), kDelta);
        assertEquals(-2.356, targetStates.rearLeft().angle().get().getRadians(), kDelta);
        assertEquals(-0.785, targetStates.rearRight().angle().get().getRadians(), kDelta);

        SwerveSetpoint prevSetpoint = new SwerveSetpoint();
        SwerveSetpoint setpoint = generator.apply(prevSetpoint, target);
        assertEquals(0, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(1, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void driveAndSpin() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        SwerveLimiter generator = new SwerveLimiter(
                unlimited, () -> 12);

        // spin fast to make the discretization effect larger
        FieldRelativeVelocity target = new FieldRelativeVelocity(5, 0, 25);
        Rotation2d theta = new Rotation2d();

        assertEquals(5, target.vxMetersPerSecond, kDelta);
        assertEquals(0, target.vyMetersPerSecond, kDelta);
        assertEquals(25, target.omegaRadiansPerSecond, kDelta);

        // this should do nothing since the limits are so high
        SwerveSetpoint prevSetpoint = new SwerveSetpoint();
        SwerveSetpoint setpoint = generator.apply(prevSetpoint, target);
        assertEquals(5, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(25, setpoint.speeds().omegaRadiansPerSecond, kDelta);

    }

    // simple accel case: are we limiting the right amount?
    @Test
    void testAccel() {
        // limit accel is 10 m/s^2
        // capsize limit is 24.5 m/s^2
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        assertEquals(24.5, limits.getMaxCapsizeAccelM_S2(), kDelta);
        SwerveLimiter swerveSetpointGenerator = new SwerveLimiter(
                limits, () -> 12);

        // initially at rest, wheels facing forward.
        FieldRelativeVelocity initialSpeeds = new FieldRelativeVelocity(0, 0, 0);

        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // initial setpoint steering is at angle zero

        // desired speed +x
        FieldRelativeVelocity desiredSpeeds = new FieldRelativeVelocity(10, 0, 0);

        // the first setpoint should be accel limited: 10 m/s^2, 0.02 sec,
        // so v = 0.2 m/s
        setpoint = swerveSetpointGenerator.apply(setpoint, desiredSpeeds);
        assertEquals(0.2, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);

        // note this says the angles are all empty which is wrong, they should be the
        // previous values.

        // after 1 second, it's going faster.
        for (int i = 0; i < 50; ++i) {
            setpoint = swerveSetpointGenerator.apply(setpoint, desiredSpeeds);
        }
        assertEquals(4.9, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testNotLimiting() {
        // high centripetal limit to stay out of the way
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        SwerveLimiter swerveSetpointGenerator = new SwerveLimiter(
                limits, () -> 12);

        // initially at rest.
        FieldRelativeVelocity initialSpeeds = new FieldRelativeVelocity(0, 0, 0);

        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is feasible, max accel = 10 * dt = 0.02 => v = 0.2
        ChassisSpFieldRelativeVelocityeeds desiredSpeeds = new FieldRelativeVelocity(0.2, 0, 0);

        setpoint = swerveSetpointGenerator.apply(setpoint, desiredSpeeds);
        assertEquals(0.2, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testLimitingALittle() {
        // high centripetal limit to stay out of the way
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        SwerveLimiter swerveSetpointGenerator = new SwerveLimiter(limits, () -> 12);

        // initially at rest.
        FieldRelativeVelocity initialSpeeds = new FieldRelativeVelocity(0, 0, 0);

        SwerveModel setpoint = new SwerveModel(initialSpeeds, initialStates);

        // desired speed is double the feasible accel so we should reach it in two
        // iterations.
        FieldRelativeVelocity desiredSpeeds = new FieldRelativeVelocity(0.4, 0, 0);

        setpoint = swerveSetpointGenerator.apply(setpoint, desiredSpeeds);
        assertEquals(0.2, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);

        setpoint = swerveSetpointGenerator.apply(setpoint, desiredSpeeds);
        assertEquals(0.4, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    /**
     * This starts full speed +x, and wants full speed +y.
     * 
     * The main purpose of this test is to print the path.
     */
    @Test
    void testCentripetal() {
        boolean DEBUG = false;
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.limiting();
        SwerveLimiter swerveSetpointGenerator = new SwerveLimiter(limits, () -> 12);

        // initially moving full speed +x
        FieldRelativeVelocity initialSpeeds = new FieldRelativeVelocity(4, 0, 0);

        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        assertEquals(4, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);

        // desired state is full speed +y
        final FieldRelativeVelocity desiredSpeeds = new FieldRelativeVelocity(0, 4, 0);

        SwerveSetpoint prev = setpoint;
        Pose2d currentPose = GeometryUtil.kPoseZero;
        if (DEBUG)
            Util.printf("i     x     y    vx    vy drive steer     ax    ay      a\n");

        // first slow from 4 m/s to 0 m/s stop at 10 m/s^2, so 0.4s
        for (int i = 0; i < 50; ++i) {
            Twist2d discrete = GeometryUtil.discretize(setpoint.speeds(), kDt);
            currentPose = currentPose.exp(discrete);
            setpoint = swerveSetpointGenerator.apply(setpoint, desiredSpeeds);

            double ax = (setpoint.speeds().vxMetersPerSecond - prev.speeds().vxMetersPerSecond)
                    / kDt;
            double ay = (setpoint.speeds().vyMetersPerSecond - prev.speeds().vyMetersPerSecond)
                    / kDt;
            double a = Math.hypot(ax, ay);

            if (DEBUG)
                Util.printf("%d %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n",
                        i, currentPose.getX(), currentPose.getY(),
                        setpoint.speeds().vxMetersPerSecond,
                        setpoint.speeds().vyMetersPerSecond,
                        setpoint.states().frontLeft().speedMetersPerSecond(),
                        setpoint.states().frontLeft().angle().get().getRadians(),
                        ax, ay, a);
            prev = setpoint;
        }

        // we end up going the right way
        assertEquals(0, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(4, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testCase4() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.decelCase();
        SwerveLimiter swerveSetpointGenerator = new SwerveLimiter(limits, () -> 12);

        // initially moving 0.5 +y
        FieldRelativeVelocity initialSpeeds = new FieldRelativeVelocity(0, 0.5, 0);

        SwerveModel setpoint = new SwerveModel(initialSpeeds, initialStates);

        // desired state is 1 +x
        final FieldRelativeVelocity desiredSpeeds = new FieldRelativeVelocity(1, 0, 0);

        setpoint = swerveSetpointGenerator.apply(setpoint, desiredSpeeds);

        // so one iteration should yield the same values as in SwerveUtilTest,
        // where the governing constraint was the steering one, s = 0.048.
        assertEquals(0.681, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0.159, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }
}
