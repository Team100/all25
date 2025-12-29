package org.team100.lib.subsystems.swerve.commands.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.sensor.gyro.MockGyro;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class ManualWithFullStateHeadingTest {
    // a bit coarser because SimHooks.stepTiming is kinda coarse.
    private static final double DELTA = 0.01;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    private Rotation2d desiredRotation = Rotation2d.kZero;

    @Test
    void testModeSwitching() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;
        ManualWithFullStateHeading m_manualWithHeading = new ManualWithFullStateHeading(
                logger,
                swerveKinodynamics,
                rotationSupplier,
                new double[] { 1.0, 1.0 });
        m_manualWithHeading.reset(new ModelSE2());

        Velocity twist1_1 = new Velocity(0, 0, 0);

        VelocitySE2 twistM_S = m_manualWithHeading.apply(new ModelSE2(), twist1_1);
        verify(0, 0, 0, twistM_S);

        // with a non-null desired rotation we're in snap mode
        assertNotNull(m_manualWithHeading.m_goal);
        desiredRotation = null;

        twist1_1 = new Velocity(0, 0, 1);
        twistM_S = m_manualWithHeading.apply(new ModelSE2(), twist1_1);
        // with a nonzero desired twist, we're out of snap mode
        assertNull(m_manualWithHeading.m_goal);

    }

    @Test
    void testNotSnapMode() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;
        ManualWithFullStateHeading m_manualWithHeading = new ManualWithFullStateHeading(
                logger,
                swerveKinodynamics,
                rotationSupplier,
                new double[] { 1.0, 1.0 });

        m_manualWithHeading.reset(new ModelSE2());

        // no desired rotation
        desiredRotation = null;

        Velocity twist1_1 = new Velocity(0, 0, 1);

        VelocitySE2 twistM_S = m_manualWithHeading.apply(
                new ModelSE2(),
                twist1_1);

        // not in snap mode
        assertNull(m_manualWithHeading.m_goal);
        verify(0, 0, 2.828, twistM_S);

        twist1_1 = new Velocity(1, 0, 0);

        twistM_S = m_manualWithHeading.apply(new ModelSE2(Pose2d.kZero, twistM_S), twist1_1);
        assertNull(m_manualWithHeading.m_goal);
        verify(1, 0, 0, twistM_S);
    }

    @Test
    void testSnapMode() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;
        ManualWithFullStateHeading m_manualWithHeading = new ManualWithFullStateHeading(
                logger,
                swerveKinodynamics,
                rotationSupplier,
                new double[] { 1.0, 0.1 });

        // facing +x
        m_manualWithHeading.reset(new ModelSE2());
        // reset means setpoint is currentpose.
        assertEquals(0, m_manualWithHeading.m_thetaSetpoint.x(), DELTA);
        assertEquals(0, m_manualWithHeading.m_thetaSetpoint.v(), DELTA);

        // face towards +y
        desiredRotation = Rotation2d.kCCW_Pi_2;
        // no user input
        final Velocity zeroVelocity = new Velocity(0, 0, 0);

        VelocitySE2 twistM_S = m_manualWithHeading.apply(
                new ModelSE2(),
                zeroVelocity);
        // in snap mode
        assertNotNull(m_manualWithHeading.m_goal);
        // but at t0 it hasn't started yet.
        // confirm the goal is what desiredRotation says.
        assertEquals(Math.PI / 2, m_manualWithHeading.m_goal.getRadians(), DELTA);
        // setpoint is the goal
        assertEquals(1.571, m_manualWithHeading.m_thetaSetpoint.x(), DELTA);
        assertEquals(0.0, m_manualWithHeading.m_thetaSetpoint.v(), DELTA);
        // k is 1 so the output is the error.
        verify(0, 0, 1.571, twistM_S);

        // let go of the pov to let the profile run.
        desiredRotation = null;

        // say we've rotated a little.
        Pose2d currentPose = new Pose2d(0, 0, new Rotation2d(0.5));
        // cheat the setpoint for the test
        m_manualWithHeading.m_thetaSetpoint = new Control100(0.5, 1);
        twistM_S = m_manualWithHeading.apply(new ModelSE2(currentPose, twistM_S), zeroVelocity);
        // setpoint is the goal
        assertEquals(0.0, m_manualWithHeading.m_thetaSetpoint.v(), DELTA);
        assertNotNull(m_manualWithHeading.m_goal);
        // some output
        verify(0, 0, 0.914, twistM_S);

        // mostly rotated
        currentPose = new Pose2d(0, 0, new Rotation2d(1.55));
        // cheat the setpoint for the test
        m_manualWithHeading.m_thetaSetpoint = new Control100(1.55, 0.2);
        twistM_S = m_manualWithHeading.apply(new ModelSE2(currentPose, twistM_S), zeroVelocity);
        // setpoint is the goal
        assertEquals(0.0, m_manualWithHeading.m_thetaSetpoint.v(), DELTA);
        assertNotNull(m_manualWithHeading.m_goal);
        //
        verify(0, 0, -0.091, twistM_S);

        // done
        currentPose = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
        m_manualWithHeading.m_thetaSetpoint = new Control100(Math.PI / 2, 0);
        twistM_S = m_manualWithHeading.apply(new ModelSE2(currentPose, twistM_S), zeroVelocity);
        assertNotNull(m_manualWithHeading.m_goal);

        // there should be no more profile to follow
        verify(0, 0, 0, twistM_S);

    }

    /** if you hold the POV the same thing should happen as above. */
    @Test
    void testSnapHeld() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;
        ManualWithFullStateHeading m_manualWithHeading = new ManualWithFullStateHeading(
                logger,
                swerveKinodynamics,
                rotationSupplier,
                new double[] { 1.0, 0.1 });

        // currently facing +x
        Pose2d currentPose = Pose2d.kZero;
        m_manualWithHeading.reset(new ModelSE2());

        // want to face towards +y
        desiredRotation = Rotation2d.kCCW_Pi_2;
        // no dtheta

        // no stick input
        Velocity twist1_1 = new Velocity(0, 0, 0);
        VelocitySE2 v = m_manualWithHeading.apply(
                new ModelSE2(currentPose, new VelocitySE2(0, 0, 0)),
                twist1_1);

        // in snap mode
        assertNotNull(m_manualWithHeading.m_goal);

        // at t0 there's not much position in the profile but there is velocity
        verify(0, 0, 1.571, v);

        // say we've rotated a little.
        currentPose = new Pose2d(0, 0, new Rotation2d(0.5));

        // cheat the setpoint for the test
        m_manualWithHeading.m_thetaSetpoint = new Control100(0.5, 1);
        v = m_manualWithHeading.apply(new ModelSE2(currentPose, v), twist1_1);
        // the setpoint is the goal
        assertEquals(0.0, m_manualWithHeading.m_thetaSetpoint.v(), DELTA);
        assertNotNull(m_manualWithHeading.m_goal);
        verify(0, 0, 0.914, v);

        // mostly rotated, so the FB controller is calm
        currentPose = new Pose2d(0, 0, new Rotation2d(1.555));
        // cheat the setpoint for the test
        m_manualWithHeading.m_thetaSetpoint = new Control100(1.555, 0.2);
        v = m_manualWithHeading.apply(new ModelSE2(currentPose, v), twist1_1);
        // the setpoint is the goal
        assertEquals(0.0, m_manualWithHeading.m_thetaSetpoint.v(), DELTA);
        assertNotNull(m_manualWithHeading.m_goal);

        // pretty close to the goal now, so low output
        verify(0, 0, -0.091, v);

        // at the setpoint
        currentPose = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
        m_manualWithHeading.m_thetaSetpoint = new Control100(Math.PI / 2, 0);
        v = m_manualWithHeading.apply(new ModelSE2(currentPose, v), twist1_1);
        assertNotNull(m_manualWithHeading.m_goal);
        // there should be no more profile to follow
        verify(0, 0, 0, v);
    }

    @Test
    void testStickyHeading() {
        Experiments.instance.testOverride(Experiment.StickyHeading, true);
        MockGyro gyro = new MockGyro();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        assertEquals(2.828, swerveKinodynamics.getMaxAngleSpeedRad_S(), DELTA);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;
        ManualWithFullStateHeading m_manualWithHeading = new ManualWithFullStateHeading(
                logger,
                swerveKinodynamics,
                rotationSupplier,
                new double[] { 1.0, 1.0 });

        // driver rotates a bit
        Velocity twist1_1 = new Velocity(0, 0, 1);

        ModelSE2 currentState = new ModelSE2(
                Pose2d.kZero,
                new VelocitySE2(0, 0, 0));
        // no POV
        desiredRotation = null;

        VelocitySE2 v = m_manualWithHeading.apply(currentState, twist1_1);
        // scale 1.0 input to max omega
        assertNull(m_manualWithHeading.m_goal);
        assertNull(m_manualWithHeading.m_thetaSetpoint);
        verify(0, 0, 2.828, v);

        // already going full speed:
        currentState = new ModelSE2(
                Pose2d.kZero,
                new VelocitySE2(0, 0, 2.828));
        // gyro indicates the correct speed
        gyro.rate = 2.828;
        v = m_manualWithHeading.apply(currentState, twist1_1);
        assertNull(m_manualWithHeading.m_goal);
        assertNull(m_manualWithHeading.m_thetaSetpoint);
        verify(0, 0, 2.828, v);

        // let go of the stick
        twist1_1 = new Velocity(0, 0, 0);
        currentState = new ModelSE2(
                Pose2d.kZero,
                new VelocitySE2(0, 0, 2.828));
        // gyro rate is still full speed.
        gyro.rate = 2.828;
        v = m_manualWithHeading.apply(currentState, twist1_1);
        // goal is the current state but at rest
        assertEquals(0.471, m_manualWithHeading.m_goal.getRadians(), DELTA);
        // setpoint is the goal
        assertEquals(0.471, m_manualWithHeading.m_thetaSetpoint.x(), DELTA);
        assertEquals(0.0, m_manualWithHeading.m_thetaSetpoint.v(), DELTA);
        // trying to stop
        verify(0, 0, -2.357, v);
    }

    @Test
    void testStickyHeading2() {
        Experiments.instance.testOverride(Experiment.StickyHeading, true);
        MockGyro gyro = new MockGyro();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        assertEquals(2.828, swerveKinodynamics.getMaxAngleSpeedRad_S(), DELTA);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;
        ManualWithFullStateHeading m_manualWithHeading = new ManualWithFullStateHeading(
                logger,
                swerveKinodynamics,
                rotationSupplier,
                new double[] { 1.0, 1.0 });

        // driver rotates a bit
        Velocity twist1_1 = new Velocity(0, 0, 1);

        ModelSE2 currentState = new ModelSE2(
                Pose2d.kZero,
                new VelocitySE2(0, 0, 0));
        // no POV
        desiredRotation = null;

        VelocitySE2 v = m_manualWithHeading.apply(currentState, twist1_1);
        // scale 1.0 input to max omega
        assertNull(m_manualWithHeading.m_goal);
        assertNull(m_manualWithHeading.m_thetaSetpoint);
        verify(0, 0, 2.828, v);

        // already going full speed:
        currentState = new ModelSE2(
                Pose2d.kZero,
                new VelocitySE2(0, 0, 2.828));
        // gyro indicates the correct speed
        gyro.rate = 2.828;
        v = m_manualWithHeading.apply(currentState, twist1_1);
        assertNull(m_manualWithHeading.m_goal);
        assertNull(m_manualWithHeading.m_thetaSetpoint);
        verify(0, 0, 2.828, v);

        // let go of the stick
        twist1_1 = new Velocity(0, 0, 0);
        currentState = new ModelSE2(
                Pose2d.kZero,
                new VelocitySE2(0, 0, 2.828));
        // gyro rate is still full speed.
        gyro.rate = 2.828;
        v = m_manualWithHeading.apply(currentState, twist1_1);
        // goal is the current state but at rest
        assertEquals(0.471, m_manualWithHeading.m_goal.getRadians(), DELTA);
        // in this version the setpoint is the goal
        assertEquals(0.471, m_manualWithHeading.m_thetaSetpoint.x(), DELTA);
        assertEquals(0.0, m_manualWithHeading.m_thetaSetpoint.v(), DELTA);
        // try very hard to stop
        verify(0, 0, -2.356, v);
    }

    /**
     * Troubleshooting the profile itself, realized the max speed was too low above
     */
    @Test
    void testProfile() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        // trapezoid adapts to max actual speed
        double kRotationSpeed = 0.5;
        assertEquals(1.414, swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed, DELTA);
        assertEquals(4.243, swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed, DELTA);
        IncrementalProfile m_profile = new TrapezoidIncrementalProfile(
                logger,
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
            u = m_profile.calculate(0.02, u, goalRaw);
        }
    }

    private void verify(double vx, double vy, double omega, VelocitySE2 v) {
        assertEquals(vx, v.x(), DELTA);
        assertEquals(vy, v.y(), DELTA);
        assertEquals(omega, v.theta(), DELTA);
    }
}
