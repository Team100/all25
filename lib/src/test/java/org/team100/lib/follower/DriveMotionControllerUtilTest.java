package org.team100.lib.follower;

import static org.junit.jupiter.api.Assertions.assertEquals;

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
import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class DriveMotionControllerUtilTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testFeedForwardAhead() {
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
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be straight ahead, no rotation.
        FollowerController controller = new FollowerController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        FieldRelativeVelocity speeds = controller.feedforward(SwerveModel.fromTimedPose(setpoint));
        assertEquals(1, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFeedForwardSideways() {
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
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be -y, robot relative, no rotation.
        FollowerController controller = new FollowerController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        FieldRelativeVelocity speeds = controller.feedforward(SwerveModel.fromTimedPose(setpoint));
        assertEquals(1, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFeedForwardTurning() {
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is tangential to the x axis but turning left
        MotionDirection motionDirection = new MotionDirection(1, 0, 1);
        // driving and turning
        double curvatureRad_M = 1;
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
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be ahead and rotating.
        FollowerController controller = new FollowerController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        FieldRelativeVelocity speeds = controller.feedforward(SwerveModel.fromTimedPose(setpoint));
        assertEquals(1, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(1, speeds.theta(), kDelta);
    }

    @Test
    void testErrorAhead() {
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
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // we're exactly on the setpoint so zero error
        FollowerController controller = new FollowerController(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02, 0.01, 0.02);
        FieldRelativeDelta positionError = controller.positionError(
                currentState,
                SwerveModel.fromTimedPose(setpoint));
        assertEquals(0, positionError.getX(), kDelta);
        assertEquals(0, positionError.getY(), kDelta);
        assertEquals(0, positionError.getRadians(), kDelta);
    }

    @Test
    void testErrorSideways() {
        // measurement is at the origin, facing down y
        Pose2d currentState = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // setpoint is +x, facing down y
        Pose2d setpointPose = new Pose2d(1, 0, GeometryUtil.kRotation90);
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
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // error is +x but robot is facing +y so error is -y
        FollowerController controller = new FollowerController(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02, 0.01, 0.02);
        FieldRelativeDelta positionError = controller.positionError(
                currentState,
                SwerveModel.fromTimedPose(setpoint));
        assertEquals(1, positionError.getX(), kDelta);
        assertEquals(0, positionError.getY(), kDelta);
        assertEquals(0, positionError.getRadians(), kDelta);
    }

    @Test
    void testFeedbackAhead() {
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
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be straight ahead, no rotation.
        FollowerController controller = new FollowerController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        FieldRelativeVelocity speeds = controller.positionFeedback(
                measurement,
                SwerveModel.fromTimedPose(setpoint));
        // we're exactly on the setpoint so zero feedback
        assertEquals(0, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFeedbackAheadPlusY() {
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
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be straight ahead, no rotation.
        FollowerController controller = new FollowerController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        FieldRelativeVelocity speeds = controller.positionFeedback(
                measurement,
                SwerveModel.fromTimedPose(setpoint));
        // setpoint should be negative y
        assertEquals(0, speeds.x(), kDelta);
        assertEquals(-1, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFeedbackAheadPlusTheta() {
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
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be straight ahead, no rotation.
        FollowerController controller = new FollowerController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        FieldRelativeVelocity speeds = controller.positionFeedback(
                measurement,
                SwerveModel.fromTimedPose(setpoint));
        // robot is on the setpoint in translation
        // but needs negative rotation
        // setpoint should be negative theta
        assertEquals(0, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(-1, speeds.theta(), kDelta);
    }

    @Test
    void testFeedbackSideways() {
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
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        FollowerController controller = new FollowerController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        FieldRelativeVelocity speeds = controller.positionFeedback(
                measurement,
                SwerveModel.fromTimedPose(setpoint));
        // on target
        assertEquals(0, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFeedbackSidewaysPlusY() {
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
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        FollowerController controller = new FollowerController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        FieldRelativeVelocity speeds = controller.positionFeedback(
                measurement,
                SwerveModel.fromTimedPose(setpoint));
        // feedback is -y field relative
        assertEquals(0, speeds.x(), kDelta);
        assertEquals(-1, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testVelocityErrorZero() {
        // measurement position doesn't matter, rotation here matches velocity below
        Pose2d currentState = new Pose2d(1, 2, new Rotation2d(Math.PI));
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
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        FollowerController controller = new FollowerController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        SwerveModel setpointModel = SwerveModel.fromTimedPose(setpoint);
        FieldRelativeVelocity error = controller.velocityError(measurement, setpointModel);
        // we're exactly on the setpoint so zero error
        assertEquals(0, error.x(), kDelta);
        assertEquals(0, error.y(), kDelta);
        assertEquals(0, error.theta(), kDelta);
    }

    @Test
    void testVelocityErrorAhead() {
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
        // measurement is the wrong velocity
        SwerveModel measurement = new SwerveModel(
                currentState,
                new FieldRelativeVelocity(0, 1, 0));
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        FollowerController controller = new FollowerController(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01, 0.02);
        SwerveModel setpointModel = SwerveModel.fromTimedPose(setpoint);
        FieldRelativeVelocity error = controller.velocityError(measurement, setpointModel);
        // error should include both components
        assertEquals(1, error.x(), kDelta);
        assertEquals(-1, error.y(), kDelta);
        assertEquals(0, error.theta(), kDelta);
    }

    @Test
    void testFullFeedbackAhead() {
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
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        FollowerController controller = new FollowerController(logger, 1, 1, 1, 1, 0.01, 0.02, 0.01, 0.02);
        FieldRelativeVelocity speeds = controller.fullFeedback(
                measurement,
                SwerveModel.fromTimedPose(setpoint));
        // we're exactly on the setpoint so zero feedback
        assertEquals(0, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFullFeedbackSideways() {
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
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        FollowerController controller = new FollowerController(logger, 1, 1, 1, 1, 0.01, 0.02, 0.01, 0.02);
        FieldRelativeVelocity speeds = controller.fullFeedback(
                measurement,
                SwerveModel.fromTimedPose(setpoint));
        // speed up
        assertEquals(0.5, speeds.x(), kDelta);
        assertEquals(0, speeds.y(), kDelta);
        assertEquals(0, speeds.theta(), kDelta);
    }

    @Test
    void testFullFeedbackSidewaysWithRotation() {
        // measurement is ahead in x and y and theta
        Pose2d currentPose = new Pose2d(0.1, 0.1,
                GeometryUtil.kRotation90.plus(new Rotation2d(0.1)));
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
        // measurement is too slow
        SwerveModel measurement = new SwerveModel(
                currentPose,
                new FieldRelativeVelocity(0.5, 0, 0));
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be straight ahead, no rotation.

        FollowerController controller = new FollowerController(logger, 1, 1, 1, 1, 0.01, 0.02, 0.01, 0.02);

        FieldRelativeVelocity positionFeedback = controller.positionFeedback(
                measurement, SwerveModel.fromTimedPose(setpoint));
        // field-relative x is ahead
        assertEquals(-0.1, positionFeedback.x(), kDelta);
        // field-relative y is ahead
        assertEquals(-0.1, positionFeedback.y(), kDelta);
        // pull back theta
        assertEquals(-0.1, positionFeedback.theta(), kDelta);

        FieldRelativeVelocity velocityFeedback = controller.velocityFeedback(
                measurement,
                SwerveModel.fromTimedPose(setpoint));

        assertEquals(0.5, velocityFeedback.x(), kDelta);
        assertEquals(0, velocityFeedback.y(), kDelta);
        assertEquals(0, velocityFeedback.theta(), kDelta);

        FieldRelativeVelocity speeds = controller.fullFeedback(
                measurement,
                SwerveModel.fromTimedPose(setpoint));
        // this is just the sum
        assertEquals(0.4, speeds.x(), kDelta);
        assertEquals(-0.1, speeds.y(), kDelta);
        assertEquals(-0.1, speeds.theta(), kDelta);
    }
}
