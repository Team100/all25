package org.team100.lib.trajectory.timing;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.path.PathFactory;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SoftRegionConstraintTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testStart() {
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(
                        new Translation2d(),
                        new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1.2);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(
                        new Translation2d(1, 0),
                        new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1.2);
        List<WaypointSE2> waypoints = List.of(w0, w1);
        // note that the first accel (10) matches the max soft accel (10)
        List<TimingConstraint> constraints = List.of(
                new ConstantConstraint(logger, 10, 2),
                new SoftRegionConstraint(w0.pose().getTranslation(), 0.1, 0.5, 10));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        Trajectory100 trajectory = planner.restToRest(waypoints);
        trajectory.dump();
    }

    @Test
    void testBoth() {
        // this forces the ctre output to the top
        Utils.isSimulation();
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(
                        new Translation2d(),
                        new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1.2);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(
                        new Translation2d(1, 0),
                        new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1.2);
        List<WaypointSE2> waypoints = List.of(w0, w1);
        // note that the first accel (10) matches the max soft accel (10)
        List<TimingConstraint> constraints = List.of(
                new ConstantConstraint(logger, 10, 10),
                new SoftRegionConstraint(w0.pose().getTranslation(), 0.1, 0.5, 10),
                new SoftRegionConstraint(w1.pose().getTranslation(), 0.1, 0.5, 10));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        // note finer path than usual in order to see the detail
        PathFactory pathFactory = new PathFactory(0.01, 0.01, 0.01, 0.1);
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        Trajectory100 trajectory = planner.restToRest(waypoints);
        trajectory.tdump();
    }

}
