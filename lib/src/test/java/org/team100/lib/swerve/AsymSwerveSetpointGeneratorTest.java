package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class AsymSwerveSetpointGeneratorTest {
    private static final double kDelta = 0.001;
    private final static double kDt = 0.02; // s
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private final static SwerveKinodynamics kKinematicLimits = SwerveKinodynamicsFactory.limiting();

    /** The setpoint generator never changes the field-relative course. */
    @Test
    void courseInvariant() {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds target = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);
        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                kKinematicLimits);

        {// motionless
            SwerveSetpoint prevSetpoint = new SwerveSetpoint();
            SwerveSetpoint setpoint = generator.generateSetpoint(
                    prevSetpoint,
                    target);
            assertTrue(GeometryUtil.getCourse(prevSetpoint.speeds()).isEmpty());
            assertTrue(GeometryUtil.getCourse(setpoint.speeds()).isEmpty());
        }
        {// at max speed, 45 to the left and spinning
            ChassisSpeeds speed = new ChassisSpeeds(2.640, 2.640, 3.733);
            SwerveModuleStates states = kKinematicLimits.toSwerveModuleStates(speed);
            SwerveSetpoint prevSetpoint = new SwerveSetpoint(speed, states);
            SwerveSetpoint setpoint = generator.generateSetpoint(
                    prevSetpoint,
                    target);
            assertEquals(Math.PI / 4, GeometryUtil.getCourse(prevSetpoint.speeds()).get().getRadians(), 1e-12);
            assertEquals(3.733, GeometryUtil.norm(prevSetpoint.speeds()), kDelta);
            assertEquals(3.733, prevSetpoint.speeds().omegaRadiansPerSecond, kDelta);
            //
            //
            // ##############
            // ## this is the bug
            // ##############
            //
            //
            assertEquals(0.7853981633974483, GeometryUtil.getCourse(setpoint.speeds()).get().getRadians(), 1e-12);
            assertEquals(3.733, GeometryUtil.norm(setpoint.speeds()), 0.2);
            assertEquals(2.505100964018383, setpoint.speeds().vxMetersPerSecond, 1e-12);
            assertEquals(2.5206083004022424, setpoint.speeds().vyMetersPerSecond, 0.2);
            assertEquals(3.733, setpoint.speeds().omegaRadiansPerSecond, 0.2);
        }
    }

    /** This is pulled from the case below, to isolate the problem. */
    @Test
    void testMakeSpeeds() {
        FieldRelativeVelocity input = new FieldRelativeVelocity(2, 0, 3.5);
        double headingRad = 0.005716666136460628;
        ChassisSpeeds prevSpeed = new ChassisSpeeds(0.16333333, 0, 0.28583333);

        ChassisSpeeds targetSpeed = SwerveKinodynamics.toInstantaneousChassisSpeeds(
                input,
                new Rotation2d(headingRad));
        assertEquals(-0.005716666136460628, GeometryUtil.getCourse(targetSpeed).get().getRadians(), 1e-12);

        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                kKinematicLimits);
        {
            // min_s == 1, works fine.
            double min_s = 1.0;
            ChassisSpeeds newSpeed = generator.makeSpeeds(prevSpeed, targetSpeed, min_s);
            // new course needs to be the same as the target course.
            assertEquals(-0.005716666136460628, GeometryUtil.getCourse(newSpeed).get().getRadians(), 1e-12);
        }
        {
            // this is the failure case.
            double min_s = 0.1;
            ChassisSpeeds newSpeed = generator.makeSpeeds(prevSpeed, targetSpeed, min_s);
            // new course needs to be the same as the target course.
            // but here we're using the lerp because the projection is too far from the
            // previous
            assertEquals(-0.0032949096257036685, GeometryUtil.getCourse(newSpeed).get().getRadians(), 1e-12);
        }
    }

    /** This is pulled from SimulatedDrivingTest, to isolate the problem. */
    @Test
    void courseInvariantRealistic() {
        FieldRelativeVelocity input = new FieldRelativeVelocity(2, 0, 3.5);

        // this is the current estimated pose heading. we've been driving for one time
        // step,
        // so we rotated a tiny bit.
        double headingRad = 0.005716666136460628;

        // not going very fast. note the previous instantaneous robot-relative speed has
        // no "y" component at all, because at the previous time, we had heading of zero
        // (and no speed either).
        ChassisSpeeds prevSpeed = new ChassisSpeeds(0.16333333, 0, 0.28583333);
        SwerveModuleStates states = kKinematicLimits.toSwerveModuleStates(prevSpeed);
        SwerveSetpoint prevSetpoint = new SwerveSetpoint(prevSpeed, states);

        // the previous course is exactly zero: this is the first time step after
        // starting.
        assertEquals(0, GeometryUtil.getCourse(prevSetpoint.speeds()).get().getRadians(), 1e-12);
        assertEquals(0.16333333, GeometryUtil.norm(prevSetpoint.speeds()), 1e-12);
        assertEquals(0.28583333, prevSetpoint.speeds().omegaRadiansPerSecond, 1e-12);

        // field-relative is +x, field-relative course is zero

        ChassisSpeeds targetSpeed = SwerveKinodynamics.toInstantaneousChassisSpeeds(
                input,
                new Rotation2d(headingRad));

        // the robot-relative course is the opposite of the heading
        assertEquals(-0.005716666136460628, GeometryUtil.getCourse(targetSpeed).get().getRadians(), 1e-6);
        // the norm is the same as the input
        assertEquals(2, GeometryUtil.norm(targetSpeed), 1e-12);
        assertEquals(1.9999673198172843, targetSpeed.vxMetersPerSecond, 1e-12);
        assertEquals(-0.011433269998955463, targetSpeed.vyMetersPerSecond, 1e-12);
        assertEquals(3.5, targetSpeed.omegaRadiansPerSecond, 1e-12);

        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                kKinematicLimits);
        SwerveSetpoint setpoint = generator.generateSetpoint(prevSetpoint, targetSpeed);

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
        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                unlimited);

        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        Rotation2d theta = new Rotation2d();
        // this is the instantaneous speed
        ChassisSpeeds target = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);

        SwerveModuleStates targetStates = unlimited.toSwerveModuleStates(target);
        assertEquals(0, target.vxMetersPerSecond, kDelta);
        assertEquals(0, target.vyMetersPerSecond, kDelta);
        assertEquals(0, target.omegaRadiansPerSecond, kDelta);
        assertEquals(0, targetStates.frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, targetStates.frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(0, targetStates.rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, targetStates.rearRight().speedMetersPerSecond(), kDelta);
        assertTrue(targetStates.frontLeft().angle().isEmpty());
        assertTrue(targetStates.frontRight().angle().isEmpty());
        assertTrue(targetStates.rearLeft().angle().isEmpty());
        assertTrue(targetStates.rearRight().angle().isEmpty());

        SwerveSetpoint prevSetpoint = new SwerveSetpoint();
        SwerveSetpoint setpoint = generator.generateSetpoint(prevSetpoint, target);
        assertEquals(0, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
        assertEquals(0, setpoint.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, setpoint.states().frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(0, setpoint.states().rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, setpoint.states().rearRight().speedMetersPerSecond(), kDelta);
        assertTrue(setpoint.states().frontLeft().angle().isEmpty());
        assertTrue(setpoint.states().frontRight().angle().isEmpty());
        assertTrue(setpoint.states().rearLeft().angle().isEmpty());
        assertTrue(setpoint.states().rearRight().angle().isEmpty());
    }

    @Test
    void driveNoOp() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                unlimited);

        FieldRelativeVelocity v = new FieldRelativeVelocity(1, 0, 0);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds target = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);

        SwerveModuleStates targetStates = unlimited.toSwerveModuleStates(target);
        assertEquals(1, target.vxMetersPerSecond, kDelta);
        assertEquals(0, target.vyMetersPerSecond, kDelta);
        assertEquals(0, target.omegaRadiansPerSecond, kDelta);
        assertEquals(1, targetStates.frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(1, targetStates.frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(1, targetStates.rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(1, targetStates.rearRight().speedMetersPerSecond(), kDelta);
        assertEquals(0, targetStates.frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(0, targetStates.frontRight().angle().get().getRadians(), kDelta);
        assertEquals(0, targetStates.rearLeft().angle().get().getRadians(), kDelta);
        assertEquals(0, targetStates.rearRight().angle().get().getRadians(), kDelta);

        SwerveSetpoint prevSetpoint = new SwerveSetpoint();
        SwerveSetpoint setpoint = generator.generateSetpoint(prevSetpoint, target);
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
        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                unlimited);

        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 1);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds target = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);

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
        SwerveSetpoint setpoint = generator.generateSetpoint(prevSetpoint, target);
        assertEquals(0, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(1, setpoint.speeds().omegaRadiansPerSecond, kDelta);
        assertEquals(0.353, setpoint.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0.353, setpoint.states().frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(0.353, setpoint.states().rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0.353, setpoint.states().rearRight().speedMetersPerSecond(), kDelta);
        assertEquals(2.356, setpoint.states().frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(0.785, setpoint.states().frontRight().angle().get().getRadians(), kDelta);
        assertEquals(-2.356, setpoint.states().rearLeft().angle().get().getRadians(), kDelta);
        assertEquals(-0.785, setpoint.states().rearRight().angle().get().getRadians(), kDelta);
    }

    @Test
    void driveAndSpin() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                unlimited);

        // spin fast to make the discretization effect larger
        FieldRelativeVelocity v = new FieldRelativeVelocity(5, 0, 25);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds target = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);

        SwerveModuleStates targetStates = unlimited.toSwerveModuleStates(target);
        assertEquals(5, target.vxMetersPerSecond, kDelta);
        assertEquals(0, target.vyMetersPerSecond, kDelta);
        assertEquals(25, target.omegaRadiansPerSecond, kDelta);
        assertEquals(5.180, targetStates.frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(12.215, targetStates.frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(7.621, targetStates.rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(13.434, targetStates.rearRight().speedMetersPerSecond(), kDelta);
        assertEquals(1.835, targetStates.frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(0.422, targetStates.frontRight().angle().get().getRadians(), kDelta);
        assertEquals(-1.749, targetStates.rearLeft().angle().get().getRadians(), kDelta);
        assertEquals(-0.592, targetStates.rearRight().angle().get().getRadians(), kDelta);

        // this should do nothing since the limits are so high
        SwerveSetpoint prevSetpoint = new SwerveSetpoint();
        SwerveSetpoint setpoint = generator.generateSetpoint(prevSetpoint, target);
        assertEquals(5, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(25, setpoint.speeds().omegaRadiansPerSecond, kDelta);
        ChassisSpeeds implied = unlimited.toChassisSpeedsWithDiscretization(setpoint.states(), 0.02);
        assertEquals(5, implied.vxMetersPerSecond, kDelta);
        assertEquals(0, implied.vyMetersPerSecond, kDelta);
        assertEquals(25, implied.omegaRadiansPerSecond, kDelta);
        assertEquals(5.180, setpoint.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(12.215, setpoint.states().frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(7.621, setpoint.states().rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(13.434, setpoint.states().rearRight().speedMetersPerSecond(), kDelta);
        assertEquals(1.835, setpoint.states().frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(0.422, setpoint.states().frontRight().angle().get().getRadians(), kDelta);
        assertEquals(-1.749, setpoint.states().rearLeft().angle().get().getRadians(), kDelta);
        assertEquals(-0.592, setpoint.states().rearRight().angle().get().getRadians(), kDelta);
    }

    // simple accel case: are we limiting the right amount?
    @Test
    void testAccel() {
        // limit accel is 10 m/s^2
        // capsize limit is 24.5 m/s^2
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        assertEquals(24.5, limits.getMaxCapsizeAccelM_S2(), kDelta);
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger,
                limits);

        // initially at rest, wheels facing forward.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // initial setpoint steering is at angle zero

        // desired speed +x
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(10, 0, 0);

        // the first setpoint should be accel limited: 10 m/s^2, 0.02 sec,
        // so v = 0.2 m/s
        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.2, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);

        // note this says the angles are all empty which is wrong, they should be the
        // previous values.

        // after 1 second, it's going faster.
        for (int i = 0; i < 50; ++i) {
            setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        }
        assertEquals(4.9, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testNotLimiting() {
        // high centripetal limit to stay out of the way
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger,
                limits);

        // initially at rest.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is feasible, max accel = 10 * dt = 0.02 => v = 0.2
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0.2, 0, 0);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.2, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testLimitingALittle() {
        // high centripetal limit to stay out of the way
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(logger, limits);

        // initially at rest.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is double the feasible accel so we should reach it in two
        // iterations.
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0.4, 0, 0);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.2, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
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
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(logger, limits);

        // initially moving full speed +x
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(4, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(4, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(4, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(4, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(4, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        assertEquals(4, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);

        // desired state is full speed +y
        final ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0, 4, 0);

        SwerveSetpoint prev = setpoint;
        Pose2d currentPose = GeometryUtil.kPoseZero;
        if (DEBUG)
            Util.printf("i     x     y    vx    vy drive steer     ax    ay      a\n");

        // first slow from 4 m/s to 0 m/s stop at 10 m/s^2, so 0.4s
        for (int i = 0; i < 50; ++i) {
            Twist2d discrete = GeometryUtil.discretize(setpoint.speeds(), kDt);
            currentPose = currentPose.exp(discrete);
            setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);

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
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(logger, limits);

        // initially moving 0.5 +y
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0.5, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0.5, Optional.of(GeometryUtil.kRotation90)),
                new SwerveModuleState100(0.5, Optional.of(GeometryUtil.kRotation90)),
                new SwerveModuleState100(0.5, Optional.of(GeometryUtil.kRotation90)),
                new SwerveModuleState100(0.5, Optional.of(GeometryUtil.kRotation90)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired state is 1 +x
        final ChassisSpeeds desiredSpeeds = new ChassisSpeeds(1, 0, 0);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);

        // so one iteration should yield the same values as in SwerveUtilTest,
        // where the governing constraint was the steering one, s = 0.048.
        assertEquals(0.681, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0.159, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

}