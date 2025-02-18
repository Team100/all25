package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

public class SwerveLimiterTest {
    private static final double kDelta = 0.001;
    private final static SwerveKinodynamics kKinematicLimits = SwerveKinodynamicsFactory.limiting();

    /** The setpoint generator never changes the field-relative course. */
    @Test
    void courseInvariant() {
        FieldRelativeVelocity target = new FieldRelativeVelocity(0, 0, 0);
        SwerveLimiter limiter = new SwerveLimiter(kKinematicLimits, () -> 12);

        {
            // motionless
            FieldRelativeVelocity prevSetpoint = new FieldRelativeVelocity(0, 0, 0);
            FieldRelativeVelocity setpoint = limiter.apply(
                    prevSetpoint,
                    target);
            assertTrue(prevSetpoint.angle().isEmpty());
            assertTrue(setpoint.angle().isEmpty());
        }
        {
            // at max speed, 45 to the left and spinning
            FieldRelativeVelocity speed = new FieldRelativeVelocity(2.640, 2.640, 3.733);
            FieldRelativeVelocity prevSetpoint = speed;
            FieldRelativeVelocity setpoint = limiter.apply(
                    prevSetpoint,
                    target);
            assertEquals(Math.PI / 4, prevSetpoint.angle().get().getRadians(), 1e-12);
            assertEquals(3.733, prevSetpoint.norm(), kDelta);
            assertEquals(3.733, prevSetpoint.theta(), kDelta);
            //
            //
            // ##############
            // ## this is the bug
            // ##############
            //
            //
            assertEquals(0.7853981633974483, setpoint.angle().get().getRadians(), 1e-12);
            assertEquals(3.733, setpoint.norm(), 0.2);
            assertEquals(2.5245058924061974, setpoint.x(), 1e-12);
            assertEquals(2.5206083004022424, setpoint.y(), 0.2);
            assertEquals(3.733, setpoint.theta(), 0.2);
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

        // the previous course is exactly zero: this is the first time step after
        // starting.
        assertEquals(0, prevSpeed.angle().get().getRadians(), 1e-12);
        assertEquals(0.16333333, prevSpeed.norm(), 1e-12);
        assertEquals(0.28583333, prevSpeed.theta(), 1e-12);

        // field-relative is +x, field-relative course is zero

        // the robot-relative course is the opposite of the heading
        assertEquals(0, targetSpeed.angle().get().getRadians(), 1e-6);
        // the norm is the same as the input
        assertEquals(2, targetSpeed.norm(), 1e-12);
        assertEquals(2, targetSpeed.x(), 1e-12);
        assertEquals(0, targetSpeed.y(), 1e-12);
        assertEquals(3.5, targetSpeed.theta(), 1e-12);

        SwerveLimiter limiter = new SwerveLimiter(kKinematicLimits, () -> 12);
        FieldRelativeVelocity setpoint = limiter.apply(prevSpeed, targetSpeed);

        // since the real heading of the robot can't change, the course here needs to
        // still be exactly
        // the opposite of the field-relative heading. if not, veering ensues.
        // here we're using the lerp because the projection is too far away,
        // i.e. this is wrong.
        assertEquals(0, setpoint.angle().get().getRadians(), 1e-6);
        assertEquals(0.3266666633333334, setpoint.norm(), 1e-12);
        assertEquals(0.5716666631110103, setpoint.theta(), 1e-12);

    }

    @Test
    void motionlessNoOp() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        SwerveLimiter limiter = new SwerveLimiter(unlimited, () -> 12);

        FieldRelativeVelocity target = new FieldRelativeVelocity(0, 0, 0);

        assertEquals(0, target.x(), kDelta);
        assertEquals(0, target.y(), kDelta);
        assertEquals(0, target.theta(), kDelta);

        FieldRelativeVelocity prevSetpoint = new FieldRelativeVelocity(0, 0, 0);
        FieldRelativeVelocity setpoint = limiter.apply(prevSetpoint, target);
        assertEquals(0, setpoint.x(), kDelta);
        assertEquals(0, setpoint.y(), kDelta);
        assertEquals(0, setpoint.theta(), kDelta);

    }

    @Test
    void driveNoOp() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        SwerveLimiter limiter = new SwerveLimiter(unlimited, () -> 12);

        FieldRelativeVelocity target = new FieldRelativeVelocity(1, 0, 0);

        assertEquals(1, target.x(), kDelta);
        assertEquals(0, target.y(), kDelta);
        assertEquals(0, target.theta(), kDelta);

        FieldRelativeVelocity prevSetpoint = new FieldRelativeVelocity(0, 0, 0);
        FieldRelativeVelocity setpoint = limiter.apply(prevSetpoint, target);
        assertEquals(1, setpoint.x(), kDelta);
        assertEquals(0, setpoint.y(), kDelta);
        assertEquals(0, setpoint.theta(), kDelta);

    }

    @Test
    void spinNoOp() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        SwerveLimiter limiter = new SwerveLimiter(unlimited, () -> 12);

        FieldRelativeVelocity target = new FieldRelativeVelocity(0, 0, 1);

        assertEquals(0, target.x(), kDelta);
        assertEquals(0, target.y(), kDelta);
        assertEquals(1, target.theta(), kDelta);

        FieldRelativeVelocity prevSetpoint = new FieldRelativeVelocity(0, 0, 0);
        FieldRelativeVelocity setpoint = limiter.apply(prevSetpoint, target);
        assertEquals(0, setpoint.x(), kDelta);
        assertEquals(0, setpoint.y(), kDelta);
        assertEquals(1, setpoint.theta(), kDelta);
    }

    @Test
    void driveAndSpin() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        SwerveLimiter limiter = new SwerveLimiter(unlimited, () -> 12);

        // spin fast to make the discretization effect larger
        FieldRelativeVelocity target = new FieldRelativeVelocity(5, 0, 25);

        assertEquals(5, target.x(), kDelta);
        assertEquals(0, target.y(), kDelta);
        assertEquals(25, target.theta(), kDelta);

        // this should do nothing since the limits are so high
        FieldRelativeVelocity prevSetpoint = new FieldRelativeVelocity(0, 0, 0);
        FieldRelativeVelocity setpoint = limiter.apply(prevSetpoint, target);
        assertEquals(5, setpoint.x(), kDelta);
        assertEquals(0, setpoint.y(), kDelta);
        assertEquals(25, setpoint.theta(), kDelta);

    }

    // simple accel case: are we limiting the right amount?
    @Test
    void testAccel() {
        // limit accel is 10 m/s^2
        // capsize limit is 24.5 m/s^2
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        assertEquals(24.5, limits.getMaxCapsizeAccelM_S2(), kDelta);
        SwerveLimiter limiter = new SwerveLimiter(limits, () -> 12);

        // initially at rest, wheels facing forward.
        FieldRelativeVelocity setpoint = new FieldRelativeVelocity(0, 0, 0);

        // initial setpoint steering is at angle zero

        // desired speed +x
        FieldRelativeVelocity desiredSpeeds = new FieldRelativeVelocity(10, 0, 0);

        // the first setpoint should be accel limited: 10 m/s^2, 0.02 sec,
        // so v = 0.2 m/s
        setpoint = limiter.apply(setpoint, desiredSpeeds);
        assertEquals(0.2, setpoint.x(), kDelta);
        assertEquals(0, setpoint.y(), kDelta);
        assertEquals(0, setpoint.theta(), kDelta);

        // note this says the angles are all empty which is wrong, they should be the
        // previous values.

        // after 1 second, it's going faster.
        for (int i = 0; i < 50; ++i) {
            setpoint = limiter.apply(setpoint, desiredSpeeds);
        }
        assertEquals(4.9, setpoint.x(), kDelta);
        assertEquals(0, setpoint.y(), kDelta);
        assertEquals(0, setpoint.theta(), kDelta);
    }

    @Test
    void testNotLimiting() {
        // high centripetal limit to stay out of the way
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        SwerveLimiter limiter = new SwerveLimiter(limits, () -> 12);

        // initially at rest.
        FieldRelativeVelocity setpoint = new FieldRelativeVelocity(0, 0, 0);

        // desired speed is feasible, max accel = 10 * dt = 0.02 => v = 0.2
        FieldRelativeVelocity desiredSpeeds = new FieldRelativeVelocity(0.2, 0, 0);

        setpoint = limiter.apply(setpoint, desiredSpeeds);
        assertEquals(0.2, setpoint.x(), kDelta);
        assertEquals(0, setpoint.y(), kDelta);
        assertEquals(0, setpoint.theta(), kDelta);
    }

    @Test
    void testLimitingALittle() {
        // high centripetal limit to stay out of the way
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        SwerveLimiter limiter = new SwerveLimiter(limits, () -> 12);

        // initially at rest.
        FieldRelativeVelocity setpoint = new FieldRelativeVelocity(0, 0, 0);

        // desired speed is double the feasible accel so we should reach it in two
        // iterations.
        FieldRelativeVelocity desiredSpeeds = new FieldRelativeVelocity(0.4, 0, 0);

        setpoint = limiter.apply(setpoint, desiredSpeeds);
        assertEquals(0.2, setpoint.x(), kDelta);
        assertEquals(0, setpoint.y(), kDelta);
        assertEquals(0, setpoint.theta(), kDelta);

        setpoint = limiter.apply(setpoint, desiredSpeeds);
        assertEquals(0.4, setpoint.x(), kDelta);
        assertEquals(0, setpoint.y(), kDelta);
        assertEquals(0, setpoint.theta(), kDelta);
    }

    @Test
    void testCase4() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.decelCase();
        SwerveLimiter limiter = new SwerveLimiter(limits, () -> 12);

        // initially moving 0.5 +y
        FieldRelativeVelocity setpoint = new FieldRelativeVelocity(0, 0.5, 0);

        // desired state is 1 +x
        final FieldRelativeVelocity desiredSpeeds = new FieldRelativeVelocity(1, 0, 0);

        setpoint = limiter.apply(setpoint, desiredSpeeds);

        assertEquals(0.146, setpoint.x(), kDelta);
        assertEquals(0.427, setpoint.y(), kDelta);
        assertEquals(0, setpoint.theta(), kDelta);
    }
}
