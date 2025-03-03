package org.team100.lib.commands.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.sensors.MockGyro;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class ManualWithProfiledHeadingTest {
    // a bit coarser because SimHooks.stepTiming is kinda coarse.
    private static final double kDelta = 0.01;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    private Rotation2d desiredRotation = Rotation2d.kZero;

    @Test
    void testModeSwitching() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        Feedback100 thetaFeedback = new PIDFeedback(logger, 3.5, 0, 0, true, 0.05, 1);
        ManualWithProfiledHeading m_manualWithHeading = new ManualWithProfiledHeading(
                logger,
                swerveKinodynamics,
                rotationSupplier,
                thetaFeedback);
        m_manualWithHeading.reset(new SwerveModel());

        DriverControl.Velocity twist1_1 = new DriverControl.Velocity(0, 0, 0);

        FieldRelativeVelocity twistM_S = m_manualWithHeading.apply(new SwerveModel(), twist1_1);
        verify(0, 0, 0, twistM_S);

        // with a non-null desired rotation we're in snap mode
        assertNotNull(m_manualWithHeading.m_goal);
        desiredRotation = null;

        twist1_1 = new DriverControl.Velocity(0, 0, 1);
        twistM_S = m_manualWithHeading.apply(new SwerveModel(), twist1_1);
        // with a nonzero desired twist, we're out of snap mode
        assertNull(m_manualWithHeading.m_goal);

    }

    @Test
    void testNotSnapMode() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        Feedback100 thetaFeedback = new PIDFeedback(logger, 3.5, 0, 0, true, 0.05, 1);
        ManualWithProfiledHeading m_manualWithHeading = new ManualWithProfiledHeading(
                logger,
                swerveKinodynamics,
                rotationSupplier,
                thetaFeedback);

        m_manualWithHeading.reset(new SwerveModel());

        // no desired rotation
        desiredRotation = null;

        DriverControl.Velocity twist1_1 = new DriverControl.Velocity(0, 0, 1);

        FieldRelativeVelocity twistM_S = m_manualWithHeading.apply(
                new SwerveModel(),
                twist1_1);

        // not in snap mode
        assertNull(m_manualWithHeading.m_goal);
        verify(0, 0, 2.828, twistM_S);

        twist1_1 = new DriverControl.Velocity(1, 0, 0);

        twistM_S = m_manualWithHeading.apply(new SwerveModel(Pose2d.kZero, twistM_S), twist1_1);
        assertNull(m_manualWithHeading.m_goal);
        verify(1, 0, 0, twistM_S);
    }

    @Test
    void testSnapMode() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        Feedback100 thetaFeedback = new PIDFeedback(logger, 3.5, 0, 0, true, 0.05, 1);
        ManualWithProfiledHeading m_manualWithHeading = new ManualWithProfiledHeading(
                logger,
                swerveKinodynamics,
                rotationSupplier,
                thetaFeedback);

        m_manualWithHeading.reset(new SwerveModel());
        // reset means setpoint is currentpose.
        assertEquals(0, m_manualWithHeading.m_thetaSetpoint.x(), kDelta);
        assertEquals(0, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);

        // face towards +y
        desiredRotation = Rotation2d.kCCW_Pi_2;
        // no user input
        final DriverControl.Velocity twist1_1 = new DriverControl.Velocity(0, 0, 0);

        // initial state is motionless
        FieldRelativeVelocity twistM_S = m_manualWithHeading.apply(
                new SwerveModel(),
                twist1_1);
        // in snap mode
        assertNotNull(m_manualWithHeading.m_goal);
        // but at t0 it hasn't started yet.
        // confirm the goal is what desiredRotation says.
        assertEquals(Math.PI / 2, m_manualWithHeading.m_goal.getRadians(), kDelta);
        // we did one calculation so setpoint is not zero
        assertEquals(0.0002, m_manualWithHeading.m_thetaSetpoint.x(), kDelta);
        // max accel is half actual max, 4.24, 0.02 => 0.0848
        assertEquals(0.017, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        // since initial state is motionless the feedback controllers apply extra
        verify(0, 0, 0.017, twistM_S);

        // let go of the pov to let the profile run.
        desiredRotation = null;

        // say we've rotated a little.
        m_manualWithHeading.m_thetaSetpoint = new Control100(0.5, 1);
        twistM_S = m_manualWithHeading.apply(
                new SwerveModel(
                        new Pose2d(0, 0, new Rotation2d(0.5)),
                        new FieldRelativeVelocity(0, 0, 0.1)),
                twist1_1);
        assertEquals(1.017, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        assertNotNull(m_manualWithHeading.m_goal);
        verify(0, 0, 1.017, twistM_S);

        // mostly rotated
        m_manualWithHeading.m_thetaSetpoint = new Control100(1.55, 0.2);
        twistM_S = m_manualWithHeading.apply(
                new SwerveModel(
                        new Pose2d(0, 0, new Rotation2d(1.55)),
                        new FieldRelativeVelocity(0, 0, 0.2)),
                twist1_1);
        assertEquals(0.183, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        assertNotNull(m_manualWithHeading.m_goal);
        // initial is setpoint, so new setpoint is a bit further, thus some feedback.
        verify(0, 0, 0.183, twistM_S);

        // done
        m_manualWithHeading.m_thetaSetpoint = new Control100(Math.PI / 2, 0);
        twistM_S = m_manualWithHeading.apply(
                new SwerveModel(
                        new Pose2d(0, 0, new Rotation2d(Math.PI / 2)),
                        new FieldRelativeVelocity(0, 0, 0)),
                twist1_1);
        assertNotNull(m_manualWithHeading.m_goal);

        // there should be no more profile to follow
        verify(0, 0, 0, twistM_S);

    }

    /** if you hold the POV the same thing should happen as above. */
    @Test
    void testSnapHeld() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        Feedback100 thetaFeedback = new PIDFeedback(logger, 3.5, 0, 0, true, 0.05, 1);
        final ManualWithProfiledHeading m_manualWithHeading = new ManualWithProfiledHeading(
                logger,
                swerveKinodynamics,
                rotationSupplier,
                thetaFeedback);

        // currently facing +x
        m_manualWithHeading.reset(new SwerveModel());

        // want to face towards +y
        desiredRotation = Rotation2d.kCCW_Pi_2;
        // no dtheta

        // no stick input
        final DriverControl.Velocity twist1_1 = new DriverControl.Velocity(0, 0, 0);
        FieldRelativeVelocity v = m_manualWithHeading.apply(
                new SwerveModel(),
                twist1_1);

        // in snap mode
        assertNotNull(m_manualWithHeading.m_goal);

        // at t0 there's not much position in the profile but there is velocity
        verify(0, 0, 0.017, v);

        // say we've rotated a little.
        m_manualWithHeading.m_thetaSetpoint = new Control100(0.5, 1);
        v = m_manualWithHeading.apply(
                new SwerveModel(
                        new Pose2d(0, 0, new Rotation2d(0.5)),
                        new FieldRelativeVelocity(0, 0, 1)),
                twist1_1);
        assertEquals(1.017, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        assertNotNull(m_manualWithHeading.m_goal);
        verify(0, 0, 1.017, v);

        // mostly rotated, so the FB controller is calm
        m_manualWithHeading.m_thetaSetpoint = new Control100(1.555, 0.2);
        v = m_manualWithHeading.apply(
                new SwerveModel(
                        new Pose2d(0, 0, new Rotation2d(1.555)),
                        new FieldRelativeVelocity(0, 0, 0.2)),
                twist1_1);
        assertEquals(0.183, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        assertNotNull(m_manualWithHeading.m_goal);

        // still want to go really fast?
        verify(0, 0, 0.183, v);

        // at the setpoint
        m_manualWithHeading.m_thetaSetpoint = new Control100(Math.PI / 2, 0);
        v = m_manualWithHeading.apply(
                new SwerveModel(
                        new Pose2d(0, 0, new Rotation2d(Math.PI / 2)),
                        new FieldRelativeVelocity(0, 0, 0)),
                twist1_1);
        assertNotNull(m_manualWithHeading.m_goal);
        // there should be no more profile to follow
        verify(0, 0, 0, v);
    }

    @Test
    void testStickyHeading() {
        Experiments.instance.testOverride(Experiment.StickyHeading, true);
        MockGyro gyro = new MockGyro();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        assertEquals(2.828, swerveKinodynamics.getMaxAngleSpeedRad_S(), kDelta);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        // NOTE no feedback here.
        Feedback100 thetaFeedback = new PIDFeedback(logger, 0, 0, 0, true, 0.05, 1);

        final ManualWithProfiledHeading m_manualWithHeading = new ManualWithProfiledHeading(
                logger,
                swerveKinodynamics,
                rotationSupplier,
                thetaFeedback);

        // driver rotates a bit
        DriverControl.Velocity control = new DriverControl.Velocity(0, 0, 1);

        SwerveModel currentState = new SwerveModel(
                Pose2d.kZero,
                new FieldRelativeVelocity(0, 0, 0));
        // no POV
        desiredRotation = null;

        FieldRelativeVelocity v = m_manualWithHeading.apply(currentState, control);
        // scale 1.0 input to max omega
        assertNull(m_manualWithHeading.m_goal);
        assertNull(m_manualWithHeading.m_thetaSetpoint);
        verify(0, 0, 2.828, v);

        // already going full speed:
        currentState = new SwerveModel(
                Pose2d.kZero,
                new FieldRelativeVelocity(0, 0, 2.828));
        // gyro indicates the correct speed
        gyro.rate = 2.828;
        v = m_manualWithHeading.apply(currentState, control);
        assertNull(m_manualWithHeading.m_goal);
        assertNull(m_manualWithHeading.m_thetaSetpoint);
        verify(0, 0, 2.828, v);

        // let go of the stick
        control = new DriverControl.Velocity(0, 0, 0);
        currentState = new SwerveModel(
                Pose2d.kZero,
                new FieldRelativeVelocity(0, 0, 2.828));
        // gyro rate is still full speed.
        gyro.rate = 2.828;

        TrapezoidProfile100 profile = m_manualWithHeading.makeProfile(0);
        // profile speed is half max.
        assertEquals(1.414, profile.getMaxVelocity(), kDelta);
        // profile accel is half max
        assertEquals(0.848, profile.getMaxAcceleration(), kDelta);

        v = m_manualWithHeading.apply(currentState, control);

        // goal is the current state but at rest; the latch calculates a goal that is
        // uses the profile max braking.
        assertEquals(-1.570, m_manualWithHeading.m_goal.getRadians(), kDelta);
        // setpoint respects velocity (though it's trying to slow down)
        // v(2.828) * dt(0.02) = 0.0566, but slowing a little
        assertEquals(0.056, m_manualWithHeading.m_thetaSetpoint.x(), kDelta);
        // braking
        assertEquals(2.811, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        // just feedforward, which is just the setpoint velocity
        verify(0, 0, 2.811, v);
    }

    @Test
    void testStickyHeading2() {
        Experiments.instance.testOverride(Experiment.StickyHeading, true);
        MockGyro gyro = new MockGyro();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        assertEquals(2.828, swerveKinodynamics.getMaxAngleSpeedRad_S(), kDelta);
        assertEquals(8.485, swerveKinodynamics.getMaxAngleAccelRad_S2(), kDelta);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        Feedback100 thetaFeedback = new PIDFeedback(logger, 3.5, 0, 0, true, 0.05, 1);

        final ManualWithProfiledHeading m_manualWithHeading = new ManualWithProfiledHeading(
                logger,
                swerveKinodynamics,
                rotationSupplier,
                thetaFeedback);

        // driver rotates a bit
        DriverControl.Velocity twist1_1 = new DriverControl.Velocity(0, 0, 1);

        SwerveModel currentState = new SwerveModel(
                Pose2d.kZero,
                new FieldRelativeVelocity(0, 0, 0));
        // no POV
        desiredRotation = null;

        FieldRelativeVelocity v = m_manualWithHeading.apply(currentState, twist1_1);
        // scale 1.0 input to max omega
        assertNull(m_manualWithHeading.m_goal);
        assertNull(m_manualWithHeading.m_thetaSetpoint);
        verify(0, 0, 2.828, v);

        // already going full speed:
        currentState = new SwerveModel(
                Pose2d.kZero,
                new FieldRelativeVelocity(0, 0, 2.828));
        // gyro indicates the correct speed
        gyro.rate = 2.828;
        v = m_manualWithHeading.apply(currentState, twist1_1);
        assertNull(m_manualWithHeading.m_goal);
        assertNull(m_manualWithHeading.m_thetaSetpoint);
        verify(0, 0, 2.828, v);

        // let go of the stick
        twist1_1 = new DriverControl.Velocity(0, 0, 0);
        currentState = new SwerveModel(
                Pose2d.kZero,
                new FieldRelativeVelocity(0, 0, 2.828));
        // gyro rate is still full speed.
        gyro.rate = 2.828;
        v = m_manualWithHeading.apply(currentState, twist1_1);

        assertEquals(-1.571, m_manualWithHeading.m_goal.getRadians(), kDelta);
        // velocity carries forward
        assertEquals(0.056, m_manualWithHeading.m_thetaSetpoint.x(), kDelta);
        // not sure how it can slow down so fast
        assertEquals(2.811, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        // includes some feedback
        verify(0, 0, 2.811, v);
    }

    /**
     * Troubleshooting the profile itself, realized the max speed was too low above
     */
    @Test
    void testProfile() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // trapezoid adapts to max actual speed
        double kRotationSpeed = 0.5;
        assertEquals(1.414, swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed, kDelta);
        assertEquals(4.243, swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed, kDelta);
        TrapezoidProfile100 m_profile = new TrapezoidProfile100(
                2.829,
                4.2,
                0.01);
        // at max heading rate
        Model100 initialRaw = new Model100(0, 2.828);
        // goal is the same but stopped, which is an overshoot profile
        Model100 goalRaw = new Model100(0, 0);
        Control100 u = initialRaw.control();

        // this produces nonsensical results. using a faster profile works fine
        // but the very slow profile is wrong somehow
        // oh it's because the current speed is faster than the max speed,
        // which never happens in reality but it should do something less dumb.

        for (int i = 0; i < 100; ++i) {
            u = m_profile.calculate(0.02, u.model(), goalRaw);
        }
    }

    private void verify(double vx, double vy, double omega, FieldRelativeVelocity v) {
        assertEquals(vx, v.x(), kDelta);
        assertEquals(vy, v.y(), kDelta);
        assertEquals(omega, v.theta(), kDelta);
    }
}
