package org.team100.lib.controller.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DeltaSE2;
import org.team100.lib.geometry.PathPoint;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.Control100;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.Model100;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.timing.TimedState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class FullStateControllerSE2Test implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testAtRest() {
        FullStateControllerSE2 c = ControllerFactorySE2.test2(logger);
        assertFalse(c.atReference());
        VelocitySE2 t = c.calculate(
                new ModelSE2(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelSE2(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ControlSE2(
                        new Control100(0, 0),
                        new Control100(0, 0),
                        new Control100(0, 0)));
        assertEquals(0, t.x(), DELTA);
        assertEquals(0, t.y(), DELTA);
        assertEquals(0, t.theta(), DELTA);
        assertTrue(c.atReference());
    }

    @Test
    void testFar() {
        FullStateControllerSE2 c = ControllerFactorySE2.test2(logger);
        assertFalse(c.atReference());
        // no velocity, no feedforward
        VelocitySE2 t = c.calculate(
                new ModelSE2(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelSE2(
                        new Model100(1, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ControlSE2(
                        new Control100(1, 0),
                        new Control100(0, 0),
                        new Control100(0, 0)));
        // 1m error so dx should be K*e = 1
        assertEquals(4, t.x(), DELTA);
        assertEquals(0, t.y(), DELTA);
        assertEquals(0, t.theta(), DELTA);
        assertFalse(c.atReference());
    }

    @Test
    void testFast() {
        FullStateControllerSE2 c = ControllerFactorySE2.test2(logger);
        assertFalse(c.atReference());
        VelocitySE2 t = c.calculate(
                new ModelSE2(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelSE2(
                        new Model100(0, 1), // produces error = 1
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ControlSE2(
                        new Control100(0, 1), // produces FF = 1
                        new Control100(0, 0),
                        new Control100(0, 0)));
        // position err is zero but velocity error is 1 and feedforward is also 1 so dx
        // should be FF + K*e = 2
        assertEquals(1.25, t.x(), DELTA);
        assertEquals(0, t.y(), DELTA);
        assertEquals(0, t.theta(), DELTA);
        assertFalse(c.atReference());
    }

    @Test
    void testOnTrack() {
        FullStateControllerSE2 c = ControllerFactorySE2.test2(logger);
        assertFalse(c.atReference());
        VelocitySE2 t = c.calculate(
                new ModelSE2(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelSE2(
                        new Model100(-1, 0.5), // position + velocity error
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ControlSE2(
                        new Control100(-1, 0.5), // velocity reference
                        new Control100(0, 0),
                        new Control100(0, 0)));
        // position and velocity controls are opposite, so just cruise
        assertEquals(-3.375, t.x(), DELTA);
        assertEquals(0, t.y(), DELTA);
        assertEquals(0, t.theta(), DELTA);
        assertFalse(c.atReference());
    }

    @Test
    void testAllAxes() {
        FullStateControllerSE2 c = ControllerFactorySE2.test2(logger);
        assertFalse(c.atReference());
        // none of these have any velocity so there's no feedforward.
        VelocitySE2 t = c.calculate(
                new ModelSE2(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelSE2(
                        new Model100(1, 0),
                        new Model100(2, 0),
                        new Model100(3, 0)),
                new ControlSE2(
                        new Control100(2, 0),
                        new Control100(4, 0),
                        new Control100(6, 0)));
        // 1m error so dx should be K*e = 1
        assertEquals(4, t.x(), DELTA);
        assertEquals(8, t.y(), DELTA);
        assertEquals(12, t.theta(), DELTA);
        assertFalse(c.atReference());
    }

    @Test
    void testRotation() {
        FullStateControllerSE2 c = ControllerFactorySE2.test2(logger);
        assertFalse(c.atReference());
        VelocitySE2 t = c.calculate(
                new ModelSE2(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(3, 0)),
                new ModelSE2(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(-3, 0)),
                new ControlSE2(
                        new Control100(0, 0),
                        new Control100(0, 0),
                        new Control100(-3, 0)));
        assertEquals(0, t.x(), DELTA);
        assertEquals(0, t.y(), DELTA);
        // we want to rotate +
        assertEquals(1.133, t.theta(), DELTA);
        assertFalse(c.atReference());
    }

    @Test
    void testErrZero() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02,
                0.01, 0.02);
        ModelSE2 measurement = new ModelSE2();
        ModelSE2 currentReference = ModelSE2
                .fromTimedState(new TimedState(
                        new PathPoint(
                                WaypointSE2.irrotational(
                                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                                0, 0),
                        0, 0, 0));
        DeltaSE2 err = controller.positionError(measurement, currentReference);
        assertEquals(0, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    @Test
    void testErrXAhead() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02,
                0.01, 0.02);
        ModelSE2 measurement = new ModelSE2(new Pose2d(1, 0, new Rotation2d()));
        ModelSE2 currentReference = ModelSE2
                .fromTimedState(new TimedState(new PathPoint(
                        WaypointSE2.irrotational(
                                new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                        0, 0), 0, 0, 0));
        DeltaSE2 err = controller.positionError(measurement, currentReference);
        assertEquals(-1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    @Test
    void testErrXBehind() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02,
                0.01, 0.02);
        ModelSE2 measurement = new ModelSE2(new Pose2d(0, 0, new Rotation2d()));
        ModelSE2 currentReference = ModelSE2
                .fromTimedState(new TimedState(new PathPoint(
                        WaypointSE2.irrotational(
                                new Pose2d(1, 0, new Rotation2d(0)), 0, 1.2),
                        0, 0), 0, 0, 0));
        DeltaSE2 err = controller.positionError(measurement, currentReference);
        assertEquals(1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    /** Rotation should not matter. */
    @Test
    void testErrXAheadWithRotation() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02,
                0.01, 0.02);
        ModelSE2 measurement = new ModelSE2(new Pose2d(1, 0, new Rotation2d(1)));
        ModelSE2 currentReference = ModelSE2
                .fromTimedState(new TimedState(new PathPoint(
                        WaypointSE2.irrotational(
                                new Pose2d(0, 0, new Rotation2d(1)), 0, 1.2),
                        0, 0), 0, 0, 0));
        DeltaSE2 err = controller.positionError(measurement, currentReference);
        assertEquals(-1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    @Test
    void testErrorAhead() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02,
                0.01, 0.02);
        // measurement is at the origin, facing ahead
        ModelSE2 measurement = new ModelSE2(new Pose2d());
        // motion is in a straight line, down the x axis

        // setpoint is also at the origin
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                0, 0);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        // we're exactly on the setpoint so zero error
        ModelSE2 currentReference = ModelSE2.fromTimedState(new TimedState(state, t, velocity, acceleration));
        DeltaSE2 positionError = controller.positionError(measurement, currentReference);
        assertEquals(0, positionError.getX(), DELTA);
        assertEquals(0, positionError.getY(), DELTA);
        assertEquals(0, positionError.getRadians(), DELTA);
    }

    @Test
    void testErrorSideways() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02,
                0.01, 0.02);
        // measurement is at the origin, facing down y
        ModelSE2 measurement = new ModelSE2(new Pose2d(0, 0, Rotation2d.kCCW_Pi_2));
        // motion is in a straight line, down the x axis
        // setpoint is +x, facing down y
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(1, 0, new Rotation2d(Math.PI / 2)), 0, 1.2),
                0, 0);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ModelSE2 currentReference = ModelSE2.fromTimedState(new TimedState(state, t, velocity, acceleration));
        DeltaSE2 positionError = controller.positionError(measurement, currentReference);
        assertEquals(1, positionError.getX(), DELTA);
        assertEquals(0, positionError.getY(), DELTA);
        assertEquals(0, positionError.getRadians(), DELTA);
    }

    @Test
    void testVelocityErrorZero() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // measurement position doesn't matter, rotation here matches velocity below
        ModelSE2 measurement = new ModelSE2(
                new Pose2d(1, 2, new Rotation2d(Math.PI)),
                new VelocitySE2(1, 0, 0));
        // motion is in a straight line, down the x axis
        // setpoint is also at the origin
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2), 0, 0);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ModelSE2 currentReference = ModelSE2.fromTimedState(new TimedState(state, t, velocity, acceleration));
        VelocitySE2 error = controller.velocityError(measurement, currentReference);
        // we're exactly on the setpoint so zero error
        assertEquals(0, error.x(), DELTA);
        assertEquals(0, error.y(), DELTA);
        assertEquals(0, error.theta(), DELTA);
    }

    @Test
    void testVelocityErrorAhead() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // measurement is at the origin, facing ahead
        // measurement is the wrong velocity
        ModelSE2 measurement = new ModelSE2(
                new Pose2d(),
                new VelocitySE2(0, 1, 0));
        // motion is in a straight line, down the x axis
        // at the origin
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                0, 0);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;

        ModelSE2 currentReference = ModelSE2.fromTimedState(new TimedState(state, t, velocity, acceleration));
        VelocitySE2 error = controller.velocityError(measurement, currentReference);
        // error should include both components
        assertEquals(1, error.x(), DELTA);
        assertEquals(-1, error.y(), DELTA);
        assertEquals(0, error.theta(), DELTA);
    }

    @Test
    void testFeedForwardAhead() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // motion is in a straight line, down the x axis
        // setpoint is also at the origin
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                0, 0);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedState setpoint = new TimedState(state, t, velocity, acceleration);
        // feedforward should be straight ahead, no rotation.
        ControlSE2 nextReference = ControlSE2.fromTimedState(setpoint);
        VelocitySE2 speeds = controller.feedforward(nextReference);
        assertEquals(1, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFeedForwardSideways() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // motion is in a straight line, down the x axis
        // setpoint is the same
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(Math.PI / 2)), 0, 1.2),
                0, 0);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedState setpoint = new TimedState(state, t, velocity, acceleration);
        // feedforward should be -y, robot relative, no rotation.
        ControlSE2 nextReference = ControlSE2.fromTimedState(setpoint);
        VelocitySE2 speeds = controller.feedforward(nextReference);
        assertEquals(1, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFeedForwardTurning() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // motion is tangential to the x axis but turning left
        // setpoint is also at the origin
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                1, 1);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ControlSE2 nextReference = ControlSE2.fromTimedState(new TimedState(state, t, velocity, acceleration));
        VelocitySE2 speeds = controller.feedforward(nextReference);
        // feedforward should be ahead and rotating.
        assertEquals(1, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(1, speeds.theta(), DELTA);
    }

    @Test
    void testFeedbackAhead() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // measurement is at the origin, facing ahead
        Pose2d currentState = new Pose2d();
        // setpoint is also at the origin
        WaypointSE2 setpointPose = WaypointSE2.irrotational(
                new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2);
        // motion is in a straight line, down the x axis
        // no curvature
        double curvatureRad_M = 0;
        PathPoint state = new PathPoint(
                setpointPose,
                0,
                curvatureRad_M);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ModelSE2 measurement = new ModelSE2(
                currentState,
                new VelocitySE2(1, 0, 0));
        // feedforward should be straight ahead, no rotation.
        ModelSE2 currentReference = ModelSE2.fromTimedState(new TimedState(state, t, velocity, acceleration));
        DeltaSE2 err = DeltaSE2.delta(measurement.pose(), currentReference.pose());
        VelocitySE2 speeds = controller.positionFeedback(err);
        // we're exactly on the setpoint so zero feedback
        assertEquals(0, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFeedbackAheadPlusY() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // measurement is plus-Y, facing ahead
        Pose2d currentState = new Pose2d(0, 1, Rotation2d.kZero);
        // setpoint is at the origin
        WaypointSE2 setpointPose = WaypointSE2.irrotational(
                new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2);
        // motion is in a straight line, down the x axis
        // no curvature
        double curvatureRad_M = 0;
        PathPoint state = new PathPoint(
                setpointPose,
                0,
                curvatureRad_M);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ModelSE2 measurement = new ModelSE2(
                currentState,
                new VelocitySE2(1, 0, 0));
        // feedforward should be straight ahead, no rotation.
        ModelSE2 currentReference = ModelSE2.fromTimedState(new TimedState(state, t, velocity, acceleration));
        DeltaSE2 err = DeltaSE2.delta(measurement.pose(), currentReference.pose());
        VelocitySE2 speeds = controller.positionFeedback(err);
        // setpoint should be negative y
        assertEquals(0, speeds.x(), DELTA);
        assertEquals(-1, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFeedbackAheadPlusTheta() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // measurement is plus-theta
        Pose2d currentState = new Pose2d(0, 0, new Rotation2d(1.0));
        // setpoint is also at the origin
        WaypointSE2 setpointPose = WaypointSE2.irrotational(
                new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2);
        // motion is in a straight line, down the x axis
        // no curvature
        double curvatureRad_M = 0;
        PathPoint state = new PathPoint(
                setpointPose,
                0,
                curvatureRad_M);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ModelSE2 measurement = new ModelSE2(
                currentState,
                new VelocitySE2(1, 0, 0));
        // feedforward should be straight ahead, no rotation.
        ModelSE2 currentReference = ModelSE2.fromTimedState(new TimedState(state, t, velocity, acceleration));
        DeltaSE2 err = DeltaSE2.delta(measurement.pose(), currentReference.pose());
        VelocitySE2 speeds = controller.positionFeedback(err);
        // robot is on the setpoint in translation
        // but needs negative rotation
        // setpoint should be negative theta
        assertEquals(0, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(-1, speeds.theta(), DELTA);
    }

    @Test
    void testFeedbackSideways() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);

        // measurement is at the origin, facing down the y axis
        Pose2d currentState = new Pose2d(0, 0, Rotation2d.kCCW_Pi_2);
        // setpoint is the same
        WaypointSE2 setpointPose = WaypointSE2.irrotational(
                new Pose2d(0, 0, new Rotation2d(Math.PI / 2)), 0, 1.2);
        // motion is in a straight line, down the x axis
        // no curvature
        double curvatureRad_M = 0;
        PathPoint state = new PathPoint(
                setpointPose,
                0,
                curvatureRad_M);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ModelSE2 measurement = new ModelSE2(
                currentState,
                new VelocitySE2(1, 0, 0));
        ModelSE2 currentReference = ModelSE2.fromTimedState(new TimedState(state, t, velocity, acceleration));
        DeltaSE2 err = DeltaSE2.delta(measurement.pose(), currentReference.pose());
        VelocitySE2 speeds = controller.positionFeedback(err);
        // on target
        assertEquals(0, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFeedbackSidewaysPlusY() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // measurement is plus-y, facing down the y axis
        Pose2d currentState = new Pose2d(0, 1, Rotation2d.kCCW_Pi_2);
        // setpoint is parallel at the origin
        WaypointSE2 setpointPose = WaypointSE2.irrotational(
                new Pose2d(0, 0, new Rotation2d(Math.PI / 2)), 0, 1.2);
        // motion is in a straight line, down the x axis
        // no curvature
        double curvatureRad_M = 0;
        PathPoint state = new PathPoint(
                setpointPose,
                0,
                curvatureRad_M);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ModelSE2 measurement = new ModelSE2(
                currentState,
                new VelocitySE2(1, 0, 0));
        ModelSE2 currentReference = ModelSE2.fromTimedState(new TimedState(state, t, velocity, acceleration));
        DeltaSE2 err = DeltaSE2.delta(measurement.pose(), currentReference.pose());
        VelocitySE2 speeds = controller.positionFeedback(err);
        // feedback is -y field relative
        assertEquals(0, speeds.x(), DELTA);
        assertEquals(-1, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFullFeedbackAhead() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 1, 1, 1, 1, 0.01, 0.02, 0.01,
                0.02);
        // measurement is at the origin, facing ahead
        Pose2d currentState = new Pose2d();
        // setpoint is also at the origin
        WaypointSE2 setpointPose = WaypointSE2.irrotational(
                new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2);
        // motion is in a straight line, down the x axis
        // no curvature
        double curvatureRad_M = 0;
        PathPoint state = new PathPoint(
                setpointPose,
                0,
                curvatureRad_M);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        // motion is on setpoint
        ModelSE2 measurement = new ModelSE2(
                currentState,
                new VelocitySE2(1, 0, 0));
        ModelSE2 currentReference = ModelSE2.fromTimedState(new TimedState(state, t, velocity, acceleration));
        DeltaSE2 perr = DeltaSE2.delta(measurement.pose(), currentReference.pose());
        VelocitySE2 verr = currentReference.velocity().minus(measurement.velocity());
        VelocitySE2 speeds = controller.fullFeedback(perr, verr);
        // we're exactly on the setpoint so zero feedback
        assertEquals(0, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFullFeedbackSideways() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 1, 1, 1, 1, 0.01, 0.02, 0.01,
                0.02);

        // measurement is at the origin, facing +y
        Pose2d currentPose = new Pose2d(0, 0, Rotation2d.kCCW_Pi_2);
        // setpoint postion is the same
        WaypointSE2 setpointPose = WaypointSE2.irrotational(
                new Pose2d(0, 0, new Rotation2d(Math.PI / 2)), 0, 1.2);
        // motion is in a straight line, down the x axis
        // no curvature
        double curvatureRad_M = 0;
        PathPoint state = new PathPoint(
                setpointPose,
                0,
                curvatureRad_M);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        // measurement is too slow
        ModelSE2 measurement = new ModelSE2(
                currentPose,
                new VelocitySE2(0.5, 0, 0));
        ModelSE2 currentReference = ModelSE2.fromTimedState(new TimedState(state, t, velocity, acceleration));
        DeltaSE2 perr = DeltaSE2.delta(measurement.pose(), currentReference.pose());
        VelocitySE2 verr = currentReference.velocity().minus(measurement.velocity());
        VelocitySE2 speeds = controller.fullFeedback(perr, verr);
        // speed up
        assertEquals(0.5, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFullFeedbackSidewaysWithRotation() {
        FullStateControllerSE2 controller = new FullStateControllerSE2(logger, 1, 1, 1, 1, 0.01, 0.02, 0.01,
                0.02);

        // measurement is ahead in x and y and theta
        // measurement is too slow
        ModelSE2 measurement = new ModelSE2(
                new Pose2d(0.1, 0.1,
                        Rotation2d.kCCW_Pi_2.plus(new Rotation2d(0.1))),
                new VelocitySE2(0.5, 0, 0));

        // setpoint postion is ahead in x and y and theta
        WaypointSE2 setpointPose = WaypointSE2.irrotational(
                new Pose2d(0, 0, new Rotation2d(Math.PI / 2)), 0, 1.2);
        // motion is in a straight line, down the x axis
        // no curvature
        double curvatureRad_M = 0;
        PathPoint state = new PathPoint(
                setpointPose,
                0,
                curvatureRad_M);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;

        ModelSE2 currentReference = ModelSE2.fromTimedState(new TimedState(state, t, velocity, acceleration));

        // feedforward should be straight ahead, no rotation.
        DeltaSE2 perr = DeltaSE2.delta(measurement.pose(), currentReference.pose());
        VelocitySE2 verr = currentReference.velocity().minus(measurement.velocity());
        VelocitySE2 positionFeedback = controller.positionFeedback(perr);
        // field-relative x is ahead
        assertEquals(-0.1, positionFeedback.x(), DELTA);
        // field-relative y is ahead
        assertEquals(-0.1, positionFeedback.y(), DELTA);
        // pull back theta
        assertEquals(-0.1, positionFeedback.theta(), DELTA);

        VelocitySE2 velocityFeedback = controller.velocityFeedback(verr);

        assertEquals(0.5, velocityFeedback.x(), DELTA);
        assertEquals(0, velocityFeedback.y(), DELTA);
        assertEquals(0, velocityFeedback.theta(), DELTA);

        VelocitySE2 speeds = controller.fullFeedback(perr, verr);
        // this is just the sum
        assertEquals(0.4, speeds.x(), DELTA);
        assertEquals(-0.1, speeds.y(), DELTA);
        assertEquals(-0.1, speeds.theta(), DELTA);
    }
}
