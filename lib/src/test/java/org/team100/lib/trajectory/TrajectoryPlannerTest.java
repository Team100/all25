package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.PathPoint;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.examples.TrajectoryExamples;
import org.team100.lib.trajectory.path.PathFactory;
import org.team100.lib.trajectory.timing.CapsizeAccelerationConstraint;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.SwerveDriveDynamicsConstraint;
import org.team100.lib.trajectory.timing.TimedState;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.timing.TrajectoryFactory;
import org.team100.lib.trajectory.timing.YawRateConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class TrajectoryPlannerTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.01;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testLinear() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1.2),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1.2));
        List<TimingConstraint> constraints = new ArrayList<>();
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        Trajectory100 t = planner.restToRest(waypoints);
        assertEquals(17, t.length());
        TimedState tp = t.getPoint(0);
        // start at zero velocity
        assertEquals(0, tp.velocityM_S(), DELTA);
        TimedState p = t.getPoint(8);
        assertEquals(0.5, p.state().waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.state().getHeadingRateRad_M(), DELTA);
    }

    @Test
    void testLinearRealistic() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1.2),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1.2));
        // these are the same as StraightLineTrajectoryTest.
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest(logger);
        List<TimingConstraint> constraints = List.of(
                new ConstantConstraint(logger, 1, 1, limits),
                new SwerveDriveDynamicsConstraint(logger, limits, 1, 1),
                new YawRateConstraint(logger, limits, 0.2),
                new CapsizeAccelerationConstraint(logger, limits, 0.2));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        Trajectory100 t = planner.restToRest(waypoints);
        assertEquals(17, t.length());
        TimedState tp = t.getPoint(0);
        assertEquals(0, tp.velocityM_S(), DELTA);
        TimedState p = t.getPoint(8);
        assertEquals(0.5, p.state().waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.state().getHeadingRateRad_M(), DELTA);
    }

    /**
     * 0.23 ms on my machine.
     * 
     * See PathFactoryTest::testPerformance().
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
        List<TimingConstraint> constraints = new ArrayList<>();
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        long startTimeNs = System.nanoTime();
        Trajectory100 t = new Trajectory100();
        // for profiling
        // final long iterations = 10000000000l;
        final long iterations = 100l;
        for (long i = 0; i < iterations; ++i) {
            t = planner.restToRest(waypoints);
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

    @Test
    void testRestToRest() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest(logger);
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood(logger);
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        ModelSE2 start = new ModelSE2(Pose2d.kZero, new VelocitySE2(0, 0, 0));
        Pose2d end = new Pose2d(1, 0, Rotation2d.kZero);
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        Trajectory100 trajectory = ex.restToRest(start.pose(), end);
        assertEquals(1.565, trajectory.duration(), DELTA);

        /** progress along trajectory */
        double m_timeS = 0;

        // initial velocity is zero.
        assertEquals(0, trajectory.sample(m_timeS).velocityM_S(), DELTA);

        double maxDriveVelocityM_S = swerveKinodynamics.getMaxDriveVelocityM_S();
        double maxDriveAccelerationM_S2 = swerveKinodynamics.getMaxDriveAccelerationM_S2();
        assertEquals(5, maxDriveVelocityM_S);
        assertEquals(10, maxDriveAccelerationM_S2);
        for (TimedState p : trajectory.getPoints()) {
            assertTrue(p.velocityM_S() - 0.001 <= maxDriveVelocityM_S,
                    String.format("%f %f", p.velocityM_S(), maxDriveVelocityM_S));
            assertTrue(p.acceleration() - 0.001 <= maxDriveAccelerationM_S2,
                    String.format("%f %f", p.acceleration(), maxDriveAccelerationM_S2));
        }
    }

    @Test
    void testMovingToRest() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest(logger);
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood(logger);
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        ModelSE2 start = new ModelSE2(Pose2d.kZero, new VelocitySE2(1, 0, 0));
        Pose2d end = new Pose2d(1, 0, Rotation2d.kZero);

        VelocitySE2 startVelocity = start.velocity();

        Translation2d full = end.getTranslation().minus(start.translation());
        Rotation2d courseToGoal = full.getAngle();
        Rotation2d startingAngle = startVelocity.angle().orElse(courseToGoal);

        // use the start velocity to adjust the first magic number.
        // divide by the distance because the spline multiplies by it
        double e1 = 2.0 * startVelocity.norm() / full.getNorm();
        Trajectory100 traj = planner.generateTrajectory(
                List.of(
                        new WaypointSE2(
                                start.pose(),
                                DirectionSE2.irrotational(startingAngle),
                                e1),
                        new WaypointSE2(
                                end,
                                DirectionSE2.irrotational(courseToGoal),
                                1.2)),
                startVelocity.norm(),
                0);
        assertEquals(1.176, traj.duration(), DELTA);
    }

    /**
     * This is a curve that goes +y, turns sharply towards +x, with a more gradual
     * curve after that.
     * 
     * Initial velocity is clamped (with a warning)
     * Max braking to the apex
     * Coast through the apex
     * Max accel for about half of the rest
     * Max decel to the end
     */
    @Test
    void test2d() {
        List<TimingConstraint> constraints = List.of(
                new ConstantConstraint(logger, 1, 1),
                new CapsizeAccelerationConstraint(logger, 1, 1));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        ModelSE2 start = new ModelSE2(Pose2d.kZero, new VelocitySE2(0, 1, 0));
        Pose2d end = new Pose2d(1, 0, Rotation2d.kZero);
        VelocitySE2 startVelocity = start.velocity();

        Translation2d full = end.getTranslation().minus(start.translation());
        Rotation2d courseToGoal = full.getAngle();
        Rotation2d startingAngle = startVelocity.angle().orElse(courseToGoal);

        // use the start velocity to adjust the first magic number.
        // divide by the distance because the spline multiplies by it
        double e1 = 2.0 * startVelocity.norm() / full.getNorm();
        Trajectory100 traj = planner.generateTrajectory(
                List.of(
                        new WaypointSE2(
                                start.pose(),
                                DirectionSE2.irrotational(startingAngle),
                                e1),
                        new WaypointSE2(
                                end,
                                DirectionSE2.irrotational(courseToGoal),
                                1.2)),
                startVelocity.norm(),
                0);
        if (DEBUG)
            traj.dump();
        assertEquals(2.741, traj.duration(), DELTA);
    }

    /**
     * Straight for 1m, approximately a quarter-circle
     * 
     * Constant velocity on the straight
     * Maximum braking just before the start of the curve
     * Declining braking as the curvature grows
     * No braking at all at the apex
     * Gradual acceleration exiting the curve
     * Maximum acceleration at the exit, to the max V
     * Constant velocity on the straight
     * 
     * These curves should be symmetric around the apex
     */
    @Test
    void test2d2() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(new Pose2d(0, 0, new Rotation2d()), new DirectionSE2(1, 0, 0), 1.3),
                new WaypointSE2(new Pose2d(1, 0, new Rotation2d()), new DirectionSE2(1, 0, 0), 1.3),
                new WaypointSE2(new Pose2d(2, 1, new Rotation2d()), new DirectionSE2(0, 1, 0), 1.3),
                new WaypointSE2(new Pose2d(2, 2, new Rotation2d()), new DirectionSE2(0, 1, 0), 1.3));

        List<TimingConstraint> constraints = List.of(
                new ConstantConstraint(logger, 1, 1),
                new CapsizeAccelerationConstraint(logger, 0.5, 1));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);

        Trajectory100 traj = planner.generateTrajectory(waypoints, 1, 1);
        if (DEBUG)
            traj.dump();
        assertEquals(4.603, traj.duration(), DELTA);
    }

    /**
     * accelerating at the start
     * slowing before the "slow zone" in the middle
     * speeding up again
     * slowing to a stop at the end
     */
    @Test
    void testVariableConstraint() {

        class ConditionalTimingConstraint implements TimingConstraint {
            @Override
            public double maxV(PathPoint state) {
                double x = state.waypoint().pose().getTranslation().getX();
                if (x < 1.5) {
                    return 2.0;
                }
                if (x < 2.5) {
                    return 1.0;
                }
                return 2.0;

            }

            @Override
            public double maxAccel(PathPoint state, double velocity) {
                return 2;
            }

            @Override
            public double maxDecel(PathPoint state, double velocity) {
                return -1;
            }
        }

        List<TimingConstraint> constraints = List.of(
                new ConditionalTimingConstraint());
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(new Pose2d(0, 0, new Rotation2d()), new DirectionSE2(1, 0, 0), 1.3),
                new WaypointSE2(new Pose2d(4, 0, new Rotation2d()), new DirectionSE2(1, 0, 0), 1.3));
        Trajectory100 traj = planner.generateTrajectory(waypoints, 0, 0);
        if (DEBUG)
            traj.dump();

    }

}
