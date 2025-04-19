package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.MockDrive;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveWithTrajectoryTest extends Fixtured implements Timeless {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TrajectoryVisualization viz = new TrajectoryVisualization(logger);
    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
    List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
    TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

    @Test
    void testTrajectoryStart() {
        Trajectory100 t = planner.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, t.sample(0).velocityM_S(), kDelta);
        SwerveController controller = SwerveControllerFactory.test(logger);

        MockDrive d = new MockDrive();
        // initially at rest
        d.m_state = new SwerveModel();


        DriveWithTrajectory c = new DriveWithTrajectory(d, controller, t, viz);

        stepTime();
        c.initialize();
        c.execute();
        // assertEquals(0.098, d.m_atRestSetpoint.x(), kDelta);
        // assertEquals(0, d.m_atRestSetpoint.y(), kDelta);
        // assertEquals(0, d.m_atRestSetpoint.theta(), kDelta);

        // we don't advance because we're still steering.
        // this next-setpoint is from "preview"
        // and our current setpoint is equal to the measurement.
        stepTime();
        c.execute();
        // assertEquals(0.098, d.m_atRestSetpoint.x(), kDelta);
        // assertEquals(0, d.m_atRestSetpoint.y(), kDelta);
        // assertEquals(0, d.m_atRestSetpoint.theta(), kDelta);

        stepTime();
        c.execute();
        assertEquals(0.102, d.m_setpoint.x(), kDelta);
        assertEquals(0, d.m_setpoint.y(), kDelta);
        assertEquals(0, d.m_setpoint.theta(), kDelta);

        // more normal driving
        stepTime();
        c.execute();
        assertEquals(0.139, d.m_setpoint.x(), kDelta);
        assertEquals(0, d.m_setpoint.y(), kDelta);
        assertEquals(0, d.m_setpoint.theta(), kDelta);

        // etc
        stepTime();
        c.execute();
        assertEquals(0.179, d.m_setpoint.x(), kDelta);
        assertEquals(0, d.m_setpoint.y(), kDelta);
        assertEquals(0, d.m_setpoint.theta(), kDelta);
    }

    @Test
    void testTrajectoryDone() {
        Trajectory100 t = planner.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, t.sample(0).velocityM_S(), kDelta);
        SwerveController controller = SwerveControllerFactory.test(logger);

        MockDrive d = new MockDrive();
        // initially at rest
        d.m_state = new SwerveModel();

        DriveWithTrajectory c = new DriveWithTrajectory(d, controller, t, viz);
        c.initialize();

        // the measurement never changes but that doesn't affect "done" as far as the
        // trajectory is concerned.
        for (int i = 0; i < 100; ++i) {
            stepTime();
            c.execute();
        }
        assertTrue(c.isDone());

    }

    /** Use a real drivetrain to observe the effect on the motors etc. */
    @Test
    void testRealDrive() {
        fixture.collection.reset();
        // this test depends on the behavior of the setpoint generator, so make sure
        // it's on (otherwise it's in whatever state the previous test left it)
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, true);
        // 1m along +x, no rotation.
        Trajectory100 trajectory = planner.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, trajectory.sample(0).velocityM_S(), kDelta);
        SwerveController controller = SwerveControllerFactory.test(logger);

        SwerveDriveSubsystem drive = fixture.drive;

        // initially at rest
        assertEquals(0, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);

        DriveWithTrajectory command = new DriveWithTrajectory(drive, controller, trajectory, viz);
        stepTime();
        command.initialize();

        command.execute();
        // but that output is not available until after takt.
        assertEquals(0, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);

        // drive normally more
        stepTime();
        command.execute();
        // this is the output from the previous takt
        assertEquals(0.02, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);

        // etc
        stepTime();
        command.execute();
        assertEquals(0.04, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);
    }

}
