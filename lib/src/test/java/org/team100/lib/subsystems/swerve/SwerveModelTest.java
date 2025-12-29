
package org.team100.lib.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.geometry.PathPoint;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.trajectory.timing.TimedState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class ModelSE2Test {
    private static final double DELTA = 0.001;

    @Test
    void testTransform() {
        Pose2d p = new Pose2d(new Translation2d(1, 1), new Rotation2d(1));
        VelocitySE2 t = new VelocitySE2(1, 1, 1);
        ModelSE2 s = new ModelSE2(p, t);
        assertEquals(1, s.x().x(), DELTA);
    }

    @Test
    void testTimedState() {
        ModelSE2 s = ModelSE2.fromTimedState(
                new TimedState(
                        new PathPoint(
                                WaypointSE2.irrotational(
                                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                                0, 0),
                        0, 0, 0));
        assertEquals(0, s.x().x(), DELTA);
        assertEquals(0, s.x().v(), DELTA);
        // assertEquals(0, s.x().a(), DELTA);
        assertEquals(0, s.y().x(), DELTA);
        assertEquals(0, s.y().v(), DELTA);
        // assertEquals(0, s.y().a(), DELTA);
    }

    @Test
    void testTimedState2() {
        ModelSE2 s = ModelSE2.fromTimedState(
                new TimedState(
                        new PathPoint(
                                WaypointSE2.irrotational(
                                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                                0, 0),
                        0, 0, 1));
        assertEquals(0, s.x().x(), DELTA);
        assertEquals(0, s.x().v(), DELTA);
        // assertEquals(1, s.x().a(), DELTA);
        assertEquals(0, s.y().x(), DELTA);
        assertEquals(0, s.y().v(), DELTA);
        // assertEquals(0, s.y().a(), DELTA);
    }

    @Test
    void testTimedState3() {
        ModelSE2 s = ModelSE2.fromTimedState(
                new TimedState(
                        new PathPoint(
                                WaypointSE2.irrotational(
                                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                                0, 0),
                        0, 1, 0));
        assertEquals(0, s.x().x(), DELTA);
        assertEquals(1, s.x().v(), DELTA);
        // assertEquals(0, s.x().a(), DELTA);
        assertEquals(0, s.y().x(), DELTA);
        assertEquals(0, s.y().v(), DELTA);
        // assertEquals(0, s.y().a(), DELTA);
    }

    /** +x motion, positive curvature => +y accel. */
    @Test
    void testTimedState4() {
        ModelSE2 s = ModelSE2.fromTimedState(
                new TimedState(
                        new PathPoint(
                                WaypointSE2.irrotational(
                                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                                0, 1),
                        0, 1, 0));
        assertEquals(0, s.x().x(), DELTA);
        assertEquals(1, s.x().v(), DELTA);
        // assertEquals(0, s.x().a(), DELTA);
        assertEquals(0, s.y().x(), DELTA);
        assertEquals(0, s.y().v(), DELTA);
        // assertEquals(1, s.y().a(), DELTA);
    }

    @Test
    void testChassisSpeeds0() {
        ModelSE2 state = new ModelSE2(
                new Pose2d(new Translation2d(0, 0), Rotation2d.kPi),
                new VelocitySE2(1, 0, 0));
        ChassisSpeeds speeds = state.chassisSpeeds();
        assertEquals(-1, speeds.vxMetersPerSecond, DELTA);
        assertEquals(0, speeds.vyMetersPerSecond, DELTA);
        assertEquals(0, speeds.omegaRadiansPerSecond, DELTA);
    }

    @Test
    void testChassisSpeeds1() {
        ModelSE2 state = new ModelSE2(
                new Pose2d(new Translation2d(0, 0), Rotation2d.kCCW_Pi_2),
                new VelocitySE2(1, 0, 1));
        ChassisSpeeds speeds = state.chassisSpeeds();
        assertEquals(0, speeds.vxMetersPerSecond, DELTA);
        assertEquals(-1, speeds.vyMetersPerSecond, DELTA);
        assertEquals(1, speeds.omegaRadiansPerSecond, DELTA);
    }
}
