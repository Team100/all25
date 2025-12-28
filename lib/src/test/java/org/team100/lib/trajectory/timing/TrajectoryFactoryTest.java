package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.PathPoint;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.path.Path100;
import org.team100.lib.trajectory.path.PathFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectoryFactoryTest {
    private static final boolean DEBUG = false;
    public static final double EPSILON = 1e-12;
    private static final double DELTA = 0.01;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    public static final List<PathPoint> WAYPOINTS = Arrays.asList(
            new PathPoint(WaypointSE2.irrotational(
                    new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2), 0, 0),
            new PathPoint(WaypointSE2.irrotational(
                    new Pose2d(24.0, 0.0, new Rotation2d(0)), 0, 1.2), 0, 0),
            new PathPoint(WaypointSE2.irrotational(
                    new Pose2d(36, 12, new Rotation2d(0)), 0, 1.2), 0, 0),
            new PathPoint(WaypointSE2.irrotational(
                    new Pose2d(60, 12, new Rotation2d(0)), 0, 1.2), 0, 0));

    public static final List<Rotation2d> HEADINGS = List.of(
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(0));

    public Trajectory100 buildAndCheckTrajectory(
            final Path100 path,
            double step_size,
            List<TimingConstraint> constraints,
            double start_vel,
            double end_vel,
            double max_vel,
            double max_acc) {
        TrajectoryFactory u = new TrajectoryFactory(constraints);
        Trajectory100 timed_traj = u.fromPath(path, start_vel, end_vel);
        checkTrajectory(timed_traj, constraints, start_vel, end_vel, max_vel, max_acc);
        return timed_traj;
    }

    public void checkTrajectory(
            final Trajectory100 traj,
            List<TimingConstraint> constraints,
            double start_vel,
            double end_vel,
            double max_vel,
            double max_acc) {
        assertFalse(traj.isEmpty());
        assertEquals(start_vel, traj.sample(0).velocityM_S(), EPSILON);
        assertEquals(end_vel, traj.getLastPoint().velocityM_S(), EPSILON);

        // Go state by state, verifying all constraints are satisfied and integration is
        // correct.
        TimedState prev_state = null;
        for (TimedState state : traj.getPoints()) {
            for (final TimingConstraint constraint : constraints) {
                assertTrue(state.velocityM_S() - EPSILON <= constraint.maxV(state.state()));
                assertTrue(state.acceleration() - EPSILON <= constraint.maxAccel(
                        state.state(), state.velocityM_S()),
                        String.format("%f %f", state.acceleration(), constraint.maxAccel(
                                state.state(), state.velocityM_S())));
                assertTrue(state.acceleration() + EPSILON >= constraint.maxDecel(state.state(), state.velocityM_S()),
                        String.format("%f %f", state.acceleration(),
                                constraint.maxDecel(state.state(), state.velocityM_S())));
            }
            if (prev_state != null) {
                assertEquals(state.velocityM_S(),
                        prev_state.velocityM_S()
                                + (state.getTimeS() - prev_state.getTimeS()) * prev_state.acceleration(),
                        EPSILON);
            }
            prev_state = state;
        }
    }

    /**
     * Turning in place does not work, but it also doesn't fail.
     */
    @Test
    void testJustTurningInPlace() {
        Path100 path = new Path100(Arrays.asList(
                new PathPoint(
                        new WaypointSE2(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                new DirectionSE2(0, 0, 1), 1),
                        1, 0),
                new PathPoint(
                        new WaypointSE2(
                                new Pose2d(0, 0, new Rotation2d(Math.PI)),
                                new DirectionSE2(0, 0, 1), 1),
                        1, 0)));

        assertEquals(0, path.getMaxDistance(), DELTA);
        assertEquals(2, path.length());
        if (DEBUG)
            System.out.printf("PATH:\n%s\n", path);

        List<TimingConstraint> constraints = new ArrayList<TimingConstraint>();
        TrajectoryFactory u = new TrajectoryFactory(constraints);
        Trajectory100 traj = u.fromPath(path, 0.0, 0.0);
        assertEquals(0, traj.duration(), DELTA);
    }

    /**
     * The path here is just four waypoints, so sharp corners.
     * 
     * The trajectory just notices velocity and acceleration along the path, so it
     * is totally infeasible at the corners.
     */
    @Test
    void testNoConstraints() {
        Path100 path = new Path100(WAYPOINTS);

        // Triangle profile.
        Trajectory100 timed_traj = buildAndCheckTrajectory(path,
                1.0,
                new ArrayList<TimingConstraint>(), 0.0, 0.0, 20.0, 5.0);
        assertEquals(4, timed_traj.length());

        // Trapezoidal profile.
        timed_traj = buildAndCheckTrajectory(path,
                1.0, new ArrayList<TimingConstraint>(),
                0.0, 0.0,
                10.0, 5.0);
        assertEquals(4, timed_traj.length());

        // Trapezoidal profile with start and end velocities.
        timed_traj = buildAndCheckTrajectory(path,
                1.0, new ArrayList<TimingConstraint>(),
                5.0, 2.0,
                10.0, 5.0);
        assertEquals(4, timed_traj.length());
    }

    /**
     * The centripetal constraint does nothing in the corners, because these paths
     * aren't realistic; the corners are ignored here.
     */
    @Test
    void testCentripetalConstraint() {
        Path100 path = new Path100(WAYPOINTS);
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest(logger);

        // Triangle profile.
        Trajectory100 timed_traj = buildAndCheckTrajectory(path,
                1.0,
                List.of(new CapsizeAccelerationConstraint(logger, limits, 1.0)), 0.0, 0.0, 20.0, 5.0);
        assertEquals(4, timed_traj.length());
        assertNotNull(timed_traj);

        // Trapezoidal profile.
        timed_traj = buildAndCheckTrajectory(path, 1.0, new ArrayList<TimingConstraint>(), 0.0, 0.0,
                10.0, 5.0);
        assertEquals(4, timed_traj.length());

        // Trapezoidal profile with start and end velocities.
        timed_traj = buildAndCheckTrajectory(path, 1.0, new ArrayList<TimingConstraint>(), 5.0, 2.0,
                10.0, 5.0);
        assertEquals(4, timed_traj.length());
    }

    @Test
    void testConditionalVelocityConstraint() {
        Path100 path = new Path100(WAYPOINTS);

        class ConditionalTimingConstraint implements TimingConstraint {
            @Override
            public double maxV(PathPoint state) {
                if (state.waypoint().pose().getTranslation().getX() >= 24.0) {
                    return 5.0;
                } else {
                    return Double.POSITIVE_INFINITY;
                }
            }

            @Override
            public double maxAccel(PathPoint state, double velocity) {
                return Double.POSITIVE_INFINITY;
            }

            @Override
            public double maxDecel(PathPoint state, double velocity) {
                return Double.NEGATIVE_INFINITY;
            }
        }

        // Trapezoidal profile.
        Trajectory100 timed_traj = buildAndCheckTrajectory(
                path, 1.0, Arrays.asList(new ConditionalTimingConstraint()), 0.0, 0.0, 10.0, 5.0);
        assertNotNull(timed_traj);
    }

    @Test
    void testConditionalAccelerationConstraint() {
        Path100 path = new Path100(WAYPOINTS);

        class ConditionalTimingConstraint implements TimingConstraint {
            @Override
            public double maxV(PathPoint state) {
                return Double.POSITIVE_INFINITY;
            }

            @Override
            public double maxAccel(PathPoint state,
                    double velocity) {
                return 10.0 / velocity;
            }

            @Override
            public double maxDecel(PathPoint state, double velocity) {
                return -10.0;
            }
        }

        // Trapezoidal profile.
        Trajectory100 timed_traj = buildAndCheckTrajectory(
                path, 1.0, Arrays.asList(new ConditionalTimingConstraint()), 0.0, 0.0, 10.0, 5.0);
        assertNotNull(timed_traj);
    }

    /**
     * 0.16 ms on my machine.
     * 
     * See PathFactoryTest::testPerformance()
     */
    @Test
    void testPerformance() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1.2),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 1),
                                new Rotation2d()),
                        new DirectionSE2(0, 1, 0), 1.2));
        long startTimeNs = System.nanoTime();
        final int iterations = 100;
        PathFactory pathFactory = new PathFactory(0.1, 0.05, 0.05, 0.2);
        Path100 path = pathFactory.fromWaypoints(waypoints);
        Trajectory100 t = new Trajectory100();
        TrajectoryFactory m_trajectoryFactory = new TrajectoryFactory(new ArrayList<>());
        for (int i = 0; i < iterations; ++i) {
            t = m_trajectoryFactory.fromPath(path, 0, 0);
        }
        long endTimeNs = System.nanoTime();
        double totalDurationMs = (endTimeNs - startTimeNs) / 1000000.0;
        if (DEBUG) {
            System.out.printf("total duration ms: %5.3f\n", totalDurationMs);
            System.out.printf("duration per iteration ms: %5.3f\n", totalDurationMs / iterations);
        }
        assertEquals(33, t.length());
        TimedState p = t.getPoint(12);
        assertEquals(0.605, p.state().waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.state().getHeadingRateRad_M(), DELTA);

    }

}
