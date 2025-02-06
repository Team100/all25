package org.team100.lib.follower;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveTrajectoryFollowerUtilTest {

    @Test
    void testErrZero() {
        SwerveModel measurement = new SwerveModel();
        TimedPose setpoint = new TimedPose(
                new Pose2dWithMotion(new Pose2d()), 0, 0, 0);
        FieldRelativeDelta err = DriveTrajectoryFollowerUtil.fieldRelativeError(measurement.pose(), setpoint);
        assertEquals(0, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    @Test
    void testErrXAhead() {
        SwerveModel measurement = new SwerveModel(new Pose2d(1, 0, new Rotation2d()));
        TimedPose setpoint = new TimedPose(
                new Pose2dWithMotion(new Pose2d()), 0, 0, 0);
        FieldRelativeDelta err = DriveTrajectoryFollowerUtil.fieldRelativeError(measurement.pose(), setpoint);
        assertEquals(-1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    @Test
    void testErrXBehind() {
        SwerveModel measurement = new SwerveModel(new Pose2d(0, 0, new Rotation2d()));
        TimedPose setpoint = new TimedPose(
                new Pose2dWithMotion(new Pose2d(1, 0, new Rotation2d())), 0, 0, 0);
        FieldRelativeDelta err = DriveTrajectoryFollowerUtil.fieldRelativeError(measurement.pose(), setpoint);
        assertEquals(1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    /** Rotation should not matter. */
    @Test
    void testErrXAheadWithRotation() {
        SwerveModel measurement = new SwerveModel(new Pose2d(1, 0, new Rotation2d(1)));
        TimedPose setpoint = new TimedPose(
                new Pose2dWithMotion(new Pose2d(0, 0, new Rotation2d(1))), 0, 0, 0);
        FieldRelativeDelta err = DriveTrajectoryFollowerUtil.fieldRelativeError(measurement.pose(), setpoint);
        assertEquals(-1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }
}
