package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.examples.TrajectoryExamples;
import org.team100.lib.trajectory.path.PathFactory;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TrajectoryRecyclerTest {
    LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void test0() {
        // this forces the ctre output to the top
        Utils.isSimulation();
        List<TimingConstraint> constraints = List.of(new ConstantConstraint(log, 1, 1));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(0, 0, new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(1, 0, new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        List<WaypointSE2> waypoints = List.of(w0, w1);
        Trajectory100 original = planner.restToRest(waypoints);
        original.dump();
        Trajectory100 recycled = TrajectoryRecycler.recycle(original, 0.1);
        recycled.dump();
    }

    /**
     * see TrajectoryPlannerTest.test2d2()
     * Straight, curve, straight.
     */
    @Test
    void test2d2() {
        // this forces the ctre output to the top
        Utils.isSimulation();
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(new Pose2d(0, 0, new Rotation2d()), new DirectionSE2(1, 0, 0), 1.3),
                new WaypointSE2(new Pose2d(1, 0, new Rotation2d()), new DirectionSE2(1, 0, 0), 1.3),
                new WaypointSE2(new Pose2d(2, 1, new Rotation2d()), new DirectionSE2(0, 1, 0), 1.3),
                new WaypointSE2(new Pose2d(2, 2, new Rotation2d()), new DirectionSE2(0, 1, 0), 1.3));

        List<TimingConstraint> constraints = List.of(
                new ConstantConstraint(log, 1, 1),
                new CapsizeAccelerationConstraint(log, 0.5, 1));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        // coarser than before
        PathFactory pathFactory = new PathFactory(0.5, 0.2, 0.2, 1);
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);

        Trajectory100 original = planner.generateTrajectory(waypoints, 1, 1);
        original.dump();
        Trajectory100 recycled = TrajectoryRecycler.recycle(original, 0.1);
        recycled.dump();

    }

}
