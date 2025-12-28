package org.team100.lib.controller.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.controller.se2.FullStateControllerSE2;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.reference.se2.TrajectoryReferenceSE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.se2.MockSubsystemSE2;
import org.team100.lib.subsystems.se2.commands.helper.VelocityReferenceControllerSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.examples.TrajectoryExamples;
import org.team100.lib.trajectory.path.Path100;
import org.team100.lib.trajectory.path.PathFactory;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.timing.TrajectoryFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ReferenceControllerSE2Test implements Timeless {
    private static final boolean DEBUG = false;

    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testTrajectoryStart() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest(logger);
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood(logger);
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        // stepTime();
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        Trajectory100 t = ex.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, t.sample(0).velocityM_S(), DELTA);
        ControllerSE2 controller = ControllerFactorySE2.test(logger);

        // initially at rest
        MockSubsystemSE2 drive = new MockSubsystemSE2(new ModelSE2());

        TrajectoryReferenceSE2 reference = new TrajectoryReferenceSE2(logger, t);
        VelocityReferenceControllerSE2 c = new VelocityReferenceControllerSE2(
                logger, drive, controller, reference);

        stepTime();
        c.execute();

        // we don't advance because we're still steering.
        // this next-setpoint is from "preview"
        // and our current setpoint is equal to the measurement.
        stepTime();
        c.execute();
        // assertEquals(0.098, drive.m_setpoint.x(), DELTA);
        // assertEquals(0, drive.m_setpoint.y(), DELTA);
        // assertEquals(0, drive.m_setpoint.theta(), DELTA);

        stepTime();
        c.execute();
        assertEquals(0.139, drive.m_setpoint.x(), DELTA);
        assertEquals(0, drive.m_setpoint.y(), DELTA);
        assertEquals(0, drive.m_setpoint.theta(), DELTA);

        // more normal driving
        stepTime();
        c.execute();
        assertEquals(0.179, drive.m_setpoint.x(), DELTA);
        assertEquals(0, drive.m_setpoint.y(), DELTA);
        assertEquals(0, drive.m_setpoint.theta(), DELTA);

        // etc
        stepTime();
        c.execute();
        assertEquals(0.221, drive.m_setpoint.x(), DELTA);
        assertEquals(0, drive.m_setpoint.y(), DELTA);
        assertEquals(0, drive.m_setpoint.theta(), DELTA);
    }

    @Test
    void testTrajectoryDone() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest(logger);
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood(logger);
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        stepTime();
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        Trajectory100 t = ex.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, t.sample(0).velocityM_S(), DELTA);
        ControllerSE2 controller = ControllerFactorySE2.test(logger);

        // initially at rest
        MockSubsystemSE2 drive = new MockSubsystemSE2(new ModelSE2());

        TrajectoryReferenceSE2 reference = new TrajectoryReferenceSE2(logger, t);
        VelocityReferenceControllerSE2 c = new VelocityReferenceControllerSE2(
                logger, drive, controller, reference);

        // the measurement never changes but that doesn't affect "done" as far as the
        // trajectory is concerned.
        for (int i = 0; i < 200; ++i) {
            stepTime();
            c.execute();
            if (DEBUG)
                System.out.printf("%s\n", drive.m_setpoint);
            // we have magically reached the end (immediately)
            drive.m_state = new ModelSE2(new Pose2d(1, 0, Rotation2d.kZero));
        }
        assertTrue(c.isDone());

    }

    @Test
    void testFieldRelativeTrajectory() {
        List<WaypointSE2> waypoints = new ArrayList<>();
        waypoints.add(new WaypointSE2(
                new Pose2d(
                        new Translation2d(),
                        Rotation2d.fromDegrees(180)),
                new DirectionSE2(1, 0, 0), 1));
        waypoints.add(new WaypointSE2(
                new Pose2d(
                        new Translation2d(100, 4),
                        Rotation2d.fromDegrees(180)),
                new DirectionSE2(1, 0, 0), 1));
        waypoints.add(new WaypointSE2(
                new Pose2d(
                        new Translation2d(196, 13),
                        Rotation2d.fromDegrees(0)),
                new DirectionSE2(1, 0, 0), 1));

        double start_vel = 0.0;
        double end_vel = 0.0;

        double stepSize = 2;

        PathFactory pathFactory = new PathFactory(stepSize, 2, 0.25, 0.1);
        Path100 path = pathFactory.fromWaypoints(waypoints);
        assertFalse(path.isEmpty());

        TrajectoryFactory u = new TrajectoryFactory(Arrays.asList());
        Trajectory100 trajectory = u.fromPath(path, start_vel, end_vel);
        if (DEBUG)
            System.out.printf("TRAJECTORY:\n%s\n", trajectory);

        FullStateControllerSE2 swerveController = new FullStateControllerSE2(
                logger,
                2.4, 2.4,
                0.1, 0.1,
                0.01, 0.02,
                0.01, 0.02);

        MockSubsystemSE2 drive = new MockSubsystemSE2(new ModelSE2());
        TrajectoryReferenceSE2 reference = new TrajectoryReferenceSE2(logger, trajectory);
        VelocityReferenceControllerSE2 referenceController = new VelocityReferenceControllerSE2(
                logger, drive, swerveController, reference);

        Pose2d pose = trajectory.sample(0).state().waypoint().pose();
        VelocitySE2 velocity = VelocitySE2.ZERO;

        double mDt = 0.02;
        int i = 0;
        while (!referenceController.isDone()) {
            if (++i > 500)
                break;
            stepTime();
            drive.m_state = new ModelSE2(pose, velocity);
            referenceController.execute();
            velocity = drive.m_recentSetpoint;
            pose = new Pose2d(
                    pose.getX() + velocity.x() * mDt,
                    pose.getY() + velocity.y() * mDt,
                    new Rotation2d(pose.getRotation().getRadians() + velocity.theta() * mDt));
            if (DEBUG)
                System.out.printf("pose %s vel %s\n", pose, velocity);
        }

        // this should be exactly right but it's not.
        assertEquals(195, pose.getTranslation().getX(), 1);
        assertEquals(13, pose.getTranslation().getY(), 0.4);
        assertEquals(0, pose.getRotation().getRadians(), 0.1);
    }

}
