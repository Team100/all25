package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.trajectory.examples.TrajectoryExamples;
import org.team100.lib.trajectory.path.PathFactory;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TrajectoryFactory;
import org.team100.lib.trajectory.timing.TimedState;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.YawRateConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectoryTest {
    private static final boolean DEBUG = false;
    LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    /**
     * Yields a straight line.
     * 
     * TrajectoryPlanner.restToRest() has several overloads: the one that takes
     * two non-holonomic poses draws a straight line between them.
     */
    @Test
    void testSimple() throws InterruptedException {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(log, 1, 0.1),
                new YawRateConstraint(log, 1, 1));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(c);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner p = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        TrajectoryExamples ex = new TrajectoryExamples(p);
        Trajectory100 t = ex.restToRest(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(10, 1, new Rotation2d()));
        new TrajectoryPlotter(0.5).plot("simple", t);
    }

    /** Turning in place does not work */
    @Test
    void testTurnInPlace() throws InterruptedException {
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(List.of(new ConstantConstraint(log, 1, 0.1)));
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner p = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        TrajectoryExamples ex = new TrajectoryExamples(p);
        Trajectory100 t = ex.restToRest(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(0, 0, new Rotation2d(1)));
        assertTrue(t.isEmpty());
    }

    @Test
    void testCircle() {
        // see HolonomicSplineTest.testCircle();
        // this is to see how to create the dtheta and curvature
        // without the spline.
        double scale = 1.3;
        WaypointSE2 p0 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), Rotation2d.k180deg),
                new DirectionSE2(0, 1, 1), scale);
        WaypointSE2 p1 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 1), Rotation2d.kCW_90deg),
                new DirectionSE2(-1, 0, 1), scale);
        WaypointSE2 p2 = new WaypointSE2(
                new Pose2d(new Translation2d(-1, 0), Rotation2d.kZero),
                new DirectionSE2(0, -1, 1), scale);
        WaypointSE2 p3 = new WaypointSE2(
                new Pose2d(new Translation2d(0, -1), Rotation2d.kCCW_90deg),
                new DirectionSE2(1, 0, 1), scale);

        List<WaypointSE2> waypoints = List.of(p0, p1, p2, p3, p0);

        List<TimingConstraint> c = List.of(
                new ConstantConstraint(log, 2, 0.5),
                new YawRateConstraint(log, 1, 1));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(c);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        Trajectory100 trajectory = planner.generateTrajectory(waypoints, 0, 0);

        TrajectoryPlotter.plot(trajectory, 0.25);
    }

    @Test
    void testDheading() {
        double scale = 1.3;
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), Rotation2d.k180deg),
                new DirectionSE2(0, 1, 1), scale);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 1), Rotation2d.kCW_90deg),
                new DirectionSE2(-1, 0, 1), scale);
        List<WaypointSE2> waypoints = List.of(w0, w1);

        List<TimingConstraint> c = List.of(
                new ConstantConstraint(log, 2, 0.5),
                new YawRateConstraint(log, 1, 1));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(c);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        Trajectory100 trajectory = planner.generateTrajectory(waypoints, 0, 0);
        double duration = trajectory.duration();
        TimedState p0 = trajectory.sample(0);
        if (DEBUG)
            System.out.println(
                    "t, intrinsic_heading_dt, heading_dt, intrinsic_ca, extrinsic_ca, extrinsic v, intrinsic v, dcourse, dcourse1");
        for (double t = 0.04; t < duration; t += 0.04) {
            TimedState p1 = trajectory.sample(t);
            Rotation2d heading0 = p0.state().waypoint().pose().getRotation();
            Rotation2d heading1 = p1.state().waypoint().pose().getRotation();
            double dheading = heading1.minus(heading0).getRadians();
            // compute time derivative of heading two ways:
            // this just compares the poses and uses the known time step
            double dheadingDt = dheading / 0.04;
            // this uses the intrinsic heading rate and the velocity
            // rad/m * m/s = rad/s
            double intrinsicDheadingDt = p0.state().getHeadingRateRad_M() * p0.velocityM_S();
            // curvature is used to compute centripetal acceleration
            // ca = v^2*curvature
            DirectionSE2 course0 = p0.state().waypoint().course();
            DirectionSE2 course1 = p1.state().waypoint().course();
            p1.state().waypoint().pose().log(p0.state().waypoint().pose());
            double dcourse1 = Metrics.translationalNorm(course1.minus(course0));
            double dcourse = course1.toRotation().minus(course0.toRotation()).getRadians();
            double intrinsicCa = p0.velocityM_S() * p0.velocityM_S() * p0.state().getCurvatureRad_M();

            if (DEBUG)
                System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
                        t, intrinsicDheadingDt, dheadingDt,
                        intrinsicCa, p0.velocityM_S(),
                        dcourse, dcourse1);
            p0 = p1;
        }

        TrajectoryPlotter.plot(trajectory, 0.25);
    }

    /**
     * Yields a curve.
     * 
     * A WaypointSE2 allows separate specification of heading (which way the
     * front of the robot is facing) and course (which way the robot is moving).
     * 
     * In this case, is facing +x, and moving +x, and it ends up moving +y but
     * facing the other way (i.e. backwards)
     */
    @Test
    void testCurved() throws InterruptedException {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(log, 2, 0.5),
                new YawRateConstraint(log, 1, 1));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(c);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner p = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 1),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(9, 9),
                                new Rotation2d(-Math.PI / 2)),
                        new DirectionSE2(0, 1, 0), 1));
        Trajectory100 t = p.restToRest(waypoints);
        new TrajectoryPlotter(0.5).plot("curved", t);
    }

    /**
     * You can specify interior waypoints as well as start and end points. Note that
     * specifying many such points will make the curve harder to calculate and
     * harder to make smooth.
     */
    @Test
    void testMultipleWaypoints() throws InterruptedException {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(log, 2, 0.5),
                new YawRateConstraint(log, 1, 1));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(c);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner p = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 1),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(5, 5),
                                new Rotation2d(-2)),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(9, 9),
                                new Rotation2d(-Math.PI / 2)),
                        new DirectionSE2(0, 1, 0), 1));
        Trajectory100 t = p.restToRest(waypoints);
        new TrajectoryPlotter(0.3).plot("multiple", t);
    }
}
