package org.team100.lib.controller.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.state.Model100;
import org.team100.lib.testing.Timeless;
import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class FullStateSwerveControllerTest implements Timeless {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testAtRest() {
        FullStateSwerveController c = SwerveControllerFactory.test2(logger);
        assertFalse(c.atReference());
        FieldRelativeVelocity t = c.calculate(
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)));
        assertEquals(0, t.x(), kDelta);
        assertEquals(0, t.y(), kDelta);
        assertEquals(0, t.theta(), kDelta);
        assertTrue(c.atReference());
    }

    @Test
    void testFar() {
        FullStateSwerveController c = SwerveControllerFactory.test2(logger);
        assertFalse(c.atReference());
        // no velocity, no feedforward
        FieldRelativeVelocity t = c.calculate(
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(1, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(1, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)));
        // 1m error so dx should be K*e = 1
        assertEquals(4, t.x(), kDelta);
        assertEquals(0, t.y(), kDelta);
        assertEquals(0, t.theta(), kDelta);
        assertFalse(c.atReference());
    }

    @Test
    void testFast() {
        FullStateSwerveController c = SwerveControllerFactory.test2(logger);
        assertFalse(c.atReference());
        FieldRelativeVelocity t = c.calculate(
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(0, 1), // produces error = 1
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(0, 1), // produces FF = 1
                        new Model100(0, 0),
                        new Model100(0, 0)));
        // position err is zero but velocity error is 1 and feedforward is also 1 so dx
        // should be FF + K*e = 2
        assertEquals(1.25, t.x(), kDelta);
        assertEquals(0, t.y(), kDelta);
        assertEquals(0, t.theta(), kDelta);
        assertFalse(c.atReference());
    }

    @Test
    void testOnTrack() {
        FullStateSwerveController c = SwerveControllerFactory.test2(logger);
        assertFalse(c.atReference());
        FieldRelativeVelocity t = c.calculate(
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(-1, 0.5), // position + velocity error
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(-1, 0.5), // velocity reference
                        new Model100(0, 0),
                        new Model100(0, 0)));
        // position and velocity controls are opposite, so just cruise
        assertEquals(-3.375, t.x(), kDelta);
        assertEquals(0, t.y(), kDelta);
        assertEquals(0, t.theta(), kDelta);
        assertFalse(c.atReference());
    }

    @Test
    void testAllAxes() {
        FullStateSwerveController c = SwerveControllerFactory.test2(logger);
        assertFalse(c.atReference());
        // none of these have any velocity so there's no feedforward.
        FieldRelativeVelocity t = c.calculate(
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(1, 0),
                        new Model100(2, 0),
                        new Model100(3, 0)),
                new SwerveModel(
                        new Model100(2, 0),
                        new Model100(4, 0),
                        new Model100(6, 0)));
        // 1m error so dx should be K*e = 1
        assertEquals(4, t.x(), kDelta);
        assertEquals(8, t.y(), kDelta);
        assertEquals(12, t.theta(), kDelta);
        assertFalse(c.atReference());
    }

    @Test
    void testRotation() {
        FullStateSwerveController c = SwerveControllerFactory.test2(logger);
        assertFalse(c.atReference());
        FieldRelativeVelocity t = c.calculate(
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(3, 0)),
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(-3, 0)),
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(-3, 0)));
        assertEquals(0, t.x(), kDelta);
        assertEquals(0, t.y(), kDelta);
        // we want to rotate +
        assertEquals(1.133, t.theta(), kDelta);
        assertFalse(c.atReference());
    }

    @Test
    void testErrZero() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02, 0.01, 0.02);
        SwerveModel measurement = new SwerveModel();
        SwerveModel currentReference = SwerveModel
                .fromTimedPose(new TimedPose(new Pose2dWithMotion(new Pose2d()), 0, 0, 0));
        FieldRelativeDelta err = controller.positionError(measurement, currentReference);
        assertEquals(0, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    @Test
    void testErrXAhead() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02, 0.01, 0.02);
        SwerveModel measurement = new SwerveModel(new Pose2d(1, 0, new Rotation2d()));
        SwerveModel currentReference = SwerveModel
                .fromTimedPose(new TimedPose(new Pose2dWithMotion(new Pose2d()), 0, 0, 0));
        FieldRelativeDelta err = controller.positionError(measurement, currentReference);
        assertEquals(-1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    @Test
    void testErrXBehind() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02, 0.01, 0.02);
        SwerveModel measurement = new SwerveModel(new Pose2d(0, 0, new Rotation2d()));
        SwerveModel currentReference = SwerveModel
                .fromTimedPose(new TimedPose(new Pose2dWithMotion(new Pose2d(1, 0, new Rotation2d())), 0, 0, 0));
        FieldRelativeDelta err = controller.positionError(measurement, currentReference);
        assertEquals(1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    /** Rotation should not matter. */
    @Test
    void testErrXAheadWithRotation() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02, 0.01, 0.02);
        SwerveModel measurement = new SwerveModel(new Pose2d(1, 0, new Rotation2d(1)));
        SwerveModel currentReference = SwerveModel
                .fromTimedPose(new TimedPose(new Pose2dWithMotion(new Pose2d(0, 0, new Rotation2d(1))), 0, 0, 0));
        FieldRelativeDelta err = controller.positionError(measurement, currentReference);
        assertEquals(-1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    @Test
    void testErrorAhead() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02, 0.01, 0.02);
        // measurement is at the origin, facing ahead
        SwerveModel measurement = new SwerveModel(new Pose2d());
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);

        // setpoint is also at the origin
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(),
                motionDirection,
                0, // no curvature
                0); // no change in curvature
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        // we're exactly on the setpoint so zero error
        SwerveModel currentReference = SwerveModel.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        FieldRelativeDelta positionError = controller.positionError(measurement, currentReference);
        assertEquals(0, positionError.getX(), kDelta);
        assertEquals(0, positionError.getY(), kDelta);
        assertEquals(0, positionError.getRadians(), kDelta);
    }

    @Test
    void testErrorSideways() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02, 0.01, 0.02);
        // measurement is at the origin, facing down y
        SwerveModel measurement = new SwerveModel(new Pose2d(0, 0, GeometryUtil.kRotation90));
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // setpoint is +x, facing down y
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(1, 0, GeometryUtil.kRotation90),
                motionDirection,
                0, // no curvature
                0); // no change in curvature
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        SwerveModel currentReference = SwerveModel.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        FieldRelativeDelta positionError = controller.positionError(measurement, currentReference);
        assertEquals(1, positionError.getX(), kDelta);
        assertEquals(0, positionError.getY(), kDelta);
        assertEquals(0, positionError.getRadians(), kDelta);
    }

    @Test
    void testVelocityErrorZero() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        // measurement position doesn't matter, rotation here matches velocity below
        SwerveModel measurement = new SwerveModel(
                new Pose2d(1, 2, new Rotation2d(Math.PI)),
                new FieldRelativeVelocity(1, 0, 0));
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // setpoint is also at the origin
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(),
                motionDirection,
                0, // no curvature
                0); // no change in curvature
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        SwerveModel currentReference = SwerveModel.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        FieldRelativeVelocity error = controller.velocityError(measurement, currentReference);
        // we're exactly on the setpoint so zero error
        assertEquals(0, error.x(), kDelta);
        assertEquals(0, error.y(), kDelta);
        assertEquals(0, error.theta(), kDelta);
    }

    @Test
    void testVelocityErrorAhead() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        // measurement is at the origin, facing ahead
        // measurement is the wrong velocity
        SwerveModel measurement = new SwerveModel(
                new Pose2d(),
                new FieldRelativeVelocity(0, 1, 0));
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // at the origin
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(),
                motionDirection,
                0, // no curvature
                0); // no change in curvature
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;

        SwerveModel currentReference = SwerveModel.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        FieldRelativeVelocity error = controller.velocityError(measurement, currentReference);
        // error should include both components
        assertEquals(1, error.x(), kDelta);
        assertEquals(-1, error.y(), kDelta);
        assertEquals(0, error.theta(), kDelta);
    }

    @Test
    void testFeedForwardAhead() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // setpoint is also at the origin
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(),
                motionDirection,
                0, // no curvature
                0);// no change in curvature
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be straight ahead, no rotation.
        SwerveModel nextReference = SwerveModel.fromTimedPose(setpoint);
        FieldRelativeVelocity speeds = controller.feedforward(nextReference);
        assertEquals(1, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFeedForwardSideways() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // setpoint is the same
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(0, 0, GeometryUtil.kRotation90),
                motionDirection,
                0, // no curvature
                0); // no change in curvature
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be -y, robot relative, no rotation.
        SwerveModel nextReference = SwerveModel.fromTimedPose(setpoint);
        FieldRelativeVelocity speeds = controller.feedforward(nextReference);
        assertEquals(1, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFeedForwardTurning() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        // motion is tangential to the x axis but turning left
        MotionDirection motionDirection = new MotionDirection(1, 0, 1);
        // setpoint is also at the origin
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(),
                motionDirection,
                1, // driving and turning
                0); // no change in curvature
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        SwerveModel nextReference = SwerveModel.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        FieldRelativeVelocity speeds = controller.feedforward(nextReference);
        // feedforward should be ahead and rotating.
        assertEquals(1, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(1, speeds.theta(), kDelta);
    }

    @Test
    void testFeedbackAhead() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        // measurement is at the origin, facing ahead
        Pose2d currentState = new Pose2d();
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        SwerveModel measurement = new SwerveModel(
                currentState,
                new FieldRelativeVelocity(1, 0, 0));
        // feedforward should be straight ahead, no rotation.
        SwerveModel currentReference = SwerveModel.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        FieldRelativeVelocity speeds = controller.positionFeedback(measurement, currentReference);
        // we're exactly on the setpoint so zero feedback
        assertEquals(0, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFeedbackAheadPlusY() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        // measurement is plus-Y, facing ahead
        Pose2d currentState = new Pose2d(0, 1, GeometryUtil.kRotationZero);
        // setpoint is at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        SwerveModel measurement = new SwerveModel(
                currentState,
                new FieldRelativeVelocity(1, 0, 0));
        // feedforward should be straight ahead, no rotation.
        SwerveModel currentReference = SwerveModel.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        FieldRelativeVelocity speeds = controller.positionFeedback(measurement, currentReference);
        // setpoint should be negative y
        assertEquals(0, speeds.x(), kDelta);
        assertEquals(-1, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFeedbackAheadPlusTheta() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        // measurement is plus-theta
        Pose2d currentState = new Pose2d(0, 0, new Rotation2d(1.0));
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        SwerveModel measurement = new SwerveModel(
                currentState,
                new FieldRelativeVelocity(1, 0, 0));
        // feedforward should be straight ahead, no rotation.
        SwerveModel currentReference = SwerveModel.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        FieldRelativeVelocity speeds = controller.positionFeedback(measurement, currentReference);
        // robot is on the setpoint in translation
        // but needs negative rotation
        // setpoint should be negative theta
        assertEquals(0, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(-1, speeds.theta(), kDelta);
    }

    @Test
    void testFeedbackSideways() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);

        // measurement is at the origin, facing down the y axis
        Pose2d currentState = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // setpoint is the same
        Pose2d setpointPose = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        SwerveModel measurement = new SwerveModel(
                currentState,
                new FieldRelativeVelocity(1, 0, 0));
        SwerveModel currentReference = SwerveModel.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        FieldRelativeVelocity speeds = controller.positionFeedback(measurement, currentReference);
        // on target
        assertEquals(0, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFeedbackSidewaysPlusY() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        // measurement is plus-y, facing down the y axis
        Pose2d currentState = new Pose2d(0, 1, GeometryUtil.kRotation90);
        // setpoint is parallel at the origin
        Pose2d setpointPose = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        SwerveModel measurement = new SwerveModel(
                currentState,
                new FieldRelativeVelocity(1, 0, 0));
        SwerveModel currentReference = SwerveModel.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        FieldRelativeVelocity speeds = controller.positionFeedback(measurement, currentReference);
        // feedback is -y field relative
        assertEquals(0, speeds.x(), kDelta);
        assertEquals(-1, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFullFeedbackAhead() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 1, 1, 1, 1, 0.01, 0.02, 0.01, 0.02);
        // measurement is at the origin, facing ahead
        Pose2d currentState = new Pose2d();
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        MotionDirection fieldRelativeMotionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                fieldRelativeMotionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        // motion is on setpoint
        SwerveModel measurement = new SwerveModel(
                currentState,
                new FieldRelativeVelocity(1, 0, 0));
        SwerveModel currentReference = SwerveModel.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        FieldRelativeVelocity speeds = controller.fullFeedback(measurement, currentReference);
        // we're exactly on the setpoint so zero feedback
        assertEquals(0, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFullFeedbackSideways() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 1, 1, 1, 1, 0.01, 0.02, 0.01, 0.02);

        // measurement is at the origin, facing +y
        Pose2d currentPose = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // setpoint postion is the same
        Pose2d setpointPose = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // motion is in a straight line, down the x axis
        MotionDirection fieldRelativeMotionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                fieldRelativeMotionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        // measurement is too slow
        SwerveModel measurement = new SwerveModel(
                currentPose,
                new FieldRelativeVelocity(0.5, 0, 0));
        SwerveModel currentReference = SwerveModel.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        FieldRelativeVelocity speeds = controller.fullFeedback(measurement, currentReference);
        // speed up
        assertEquals(0.5, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFullFeedbackSidewaysWithRotation() {
        FullStateSwerveController controller = new FullStateSwerveController(logger, 1, 1, 1, 1, 0.01, 0.02, 0.01, 0.02);

        // measurement is ahead in x and y and theta
        // measurement is too slow
        SwerveModel measurement = new SwerveModel(
                new Pose2d(0.1, 0.1,
                        GeometryUtil.kRotation90.plus(new Rotation2d(0.1))),
                new FieldRelativeVelocity(0.5, 0, 0));

        // setpoint postion is ahead in x and y and theta
        Pose2d setpointPose = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // motion is in a straight line, down the x axis
        MotionDirection fieldRelativeMotionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                fieldRelativeMotionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;

        SwerveModel currentReference = SwerveModel.fromTimedPose(new TimedPose(state, t, velocity, acceleration));

        // feedforward should be straight ahead, no rotation.

        FieldRelativeVelocity positionFeedback = controller.positionFeedback(
                measurement, currentReference);
        // field-relative x is ahead
        assertEquals(-0.1, positionFeedback.x(), kDelta);
        // field-relative y is ahead
        assertEquals(-0.1, positionFeedback.y(), kDelta);
        // pull back theta
        assertEquals(-0.1, positionFeedback.theta(), kDelta);

        FieldRelativeVelocity velocityFeedback = controller.velocityFeedback(
                measurement,
                currentReference);

        assertEquals(0.5, velocityFeedback.x(), kDelta);
        assertEquals(0, velocityFeedback.y(), kDelta);
        assertEquals(0, velocityFeedback.theta(), kDelta);

        FieldRelativeVelocity speeds = controller.fullFeedback(
                measurement,
                currentReference);
        // this is just the sum
        assertEquals(0.4, speeds.x(), kDelta);
        assertEquals(-0.1, speeds.y(), kDelta);
        assertEquals(-0.1, speeds.theta(), kDelta);
    }
}
