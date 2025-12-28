package org.team100.lib.trajectory.timing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.PathPoint;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.path.Path100;
import org.team100.lib.trajectory.path.PathFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Verify that trajectory schedule generation yields a realistic profile.
 * 
 * https://docs.google.com/spreadsheets/d/16UUCCz-qcPz_YZMnsJnVkVO1KGp5zHCOVo7EoJct2nA/edit?gid=0#gid=0
 */
public class TrajectoryVelocityProfileTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    // A five-meter straight line.
    private static final PathPoint[] WAYPOINTS = new PathPoint[] {
            new PathPoint(WaypointSE2.irrotational(
                    new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2), 0, 0),
            new PathPoint(WaypointSE2.irrotational(
                    new Pose2d(2.5, 0, new Rotation2d(0)), 0, 1.2), 0, 0),
            new PathPoint(WaypointSE2.irrotational(
                    new Pose2d(5, 0, new Rotation2d(0)), 0, 1.2), 0, 0) };

    private static List<WaypointSE2> waypointList = Arrays.asList(WAYPOINTS).stream().map(p -> p.waypoint()).toList();
    private static PathFactory pathFactory = new PathFactory(0.1, 0.1, 0.1, 0.1);
    private static Path100 path = pathFactory.fromWaypoints(waypointList);

    /**
     * Default max accel and velocity makes a very fast triangle profile.
     */
    @Test
    void testNoConstraint() {
        List<TimingConstraint> constraints = new ArrayList<TimingConstraint>();
        TrajectoryFactory u = new TrajectoryFactory(constraints);
        Trajectory100 traj = u.fromPath(path, 0, 0);
        if (DEBUG)
            traj.dump();
    }

    /**
     * This produces a trapezoid with the correct cruise (3.5) and accel/decel the
     * same (10)
     */
    @Test
    void testConstantConstraint() {
        // somewhat realistic numbers
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTrajectoryTimingTest(logger);
        List<TimingConstraint> constraints = List.of(new ConstantConstraint(logger, 1, 1, limits));
        TrajectoryFactory u = new TrajectoryFactory(constraints);
        Trajectory100 traj = u.fromPath(path, 0, 0);
        if (DEBUG)
            traj.dump();
    }

    /**
     * This produces the desired current-limited exponential shape for acceleration,
     * and faster decel at the end.
     * 
     * https://docs.google.com/spreadsheets/d/1sbB-zTBUjRRlWHaWXe-V1ZDhAZCFwItVVO1x3LmZ4B4/edit?gid=104506786#gid=104506786
     */
    @Test
    void testSwerveConstraint() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTrajectoryTimingTest(logger);
        List<TimingConstraint> constraints = List.of(new SwerveDriveDynamicsConstraint(logger, limits, 1, 1));
        TrajectoryFactory u = new TrajectoryFactory(constraints);
        Trajectory100 traj = u.fromPath(path, 0, 0);
        if (DEBUG)
            traj.dump();
    }

    /**
     * Realistic, cruses at 3.5 (which is right)
     * 
     * https://docs.google.com/spreadsheets/d/1sbB-zTBUjRRlWHaWXe-V1ZDhAZCFwItVVO1x3LmZ4B4/edit?gid=1802036642#gid=1802036642
     */
    @Test
    void testAuto() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTrajectoryTimingTest(logger);
        TimingConstraintFactory timing = new TimingConstraintFactory(limits);
        List<TimingConstraint> constraints = timing.testAuto(logger);
        TrajectoryFactory u = new TrajectoryFactory(constraints);
        Trajectory100 traj = u.fromPath(path, 0, 0);
        if (DEBUG)
            traj.dump();
    }
}
