package org.team100.lib.subsystems.se2.commands.test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.localization.FreshSwerveEstimate;
import org.team100.lib.localization.NudgingVisionUpdater;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.SimulatedGyro;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.se2.MockSubsystemSE2;
import org.team100.lib.subsystems.se2.commands.test.DriveWithTrajectory;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.SwerveLocal;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.examples.TrajectoryExamples;
import org.team100.lib.trajectory.path.PathFactory;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.timing.TrajectoryFactory;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveWithTrajectoryTest implements Timeless {

    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final LoggerFactory fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TrajectoryVisualization viz = new TrajectoryVisualization(logger);

    @Test
    void testTrajectoryStart() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest(logger);
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood(logger);
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        Trajectory100 t = ex.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, t.sample(0).velocityM_S(), DELTA);
        ControllerSE2 controller = ControllerFactorySE2.test(logger);

        // initially at rest
        MockSubsystemSE2 d = new MockSubsystemSE2(new ModelSE2());

        DriveWithTrajectory c = new DriveWithTrajectory(logger, d, controller, t, viz);

        stepTime();
        c.initialize();
        c.execute();
        // assertEquals(0.098, d.m_atRestSetpoint.x(), DELTA);
        // assertEquals(0, d.m_atRestSetpoint.y(), DELTA);
        // assertEquals(0, d.m_atRestSetpoint.theta(), DELTA);

        // we don't advance because we're still steering.
        // this next-setpoint is from "preview"
        // and our current setpoint is equal to the measurement.
        stepTime();
        c.execute();
        // assertEquals(0.098, d.m_atRestSetpoint.x(), DELTA);
        // assertEquals(0, d.m_atRestSetpoint.y(), DELTA);
        // assertEquals(0, d.m_atRestSetpoint.theta(), DELTA);

        stepTime();
        c.execute();
        assertEquals(0.125, d.m_setpoint.x(), DELTA);
        assertEquals(0, d.m_setpoint.y(), DELTA);
        assertEquals(0, d.m_setpoint.theta(), DELTA);

        // more normal driving
        stepTime();
        c.execute();
        assertEquals(0.171, d.m_setpoint.x(), DELTA);
        assertEquals(0, d.m_setpoint.y(), DELTA);
        assertEquals(0, d.m_setpoint.theta(), DELTA);

        // etc
        stepTime();
        c.execute();
        assertEquals(0.217, d.m_setpoint.x(), DELTA);
        assertEquals(0, d.m_setpoint.y(), DELTA);
        assertEquals(0, d.m_setpoint.theta(), DELTA);
    }

    @Test
    void testTrajectoryDone() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest(logger);
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood(logger);
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        Trajectory100 t = ex.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, t.sample(0).velocityM_S(), DELTA);
        ControllerSE2 controller = ControllerFactorySE2.test(logger);

        // initially at rest
        MockSubsystemSE2 d = new MockSubsystemSE2(new ModelSE2());

        DriveWithTrajectory c = new DriveWithTrajectory(logger, d, controller, t, viz);
        c.initialize();

        for (int i = 0; i < 100; ++i) {
            stepTime();
            c.execute();
            // we have magically reached the end
            d.m_state = new ModelSE2(new Pose2d(1, 0, Rotation2d.kZero));
        }
        assertTrue(c.isDone());

    }

    /**
     * Use a real drivetrain to observe the effect on the motors etc.
     * 
     * @throws IOException
     */
    @Test
    void testRealDrive() throws IOException {

        // this test depends on the behavior of the setpoint generator, so make sure
        // it's on (otherwise it's in whatever state the previous test left it)
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, true);
        // 1m along +x, no rotation.
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest(logger);
        SwerveModuleCollection collection = SwerveModuleCollection.get(logger, 10, 20, swerveKinodynamics);
        collection.reset();
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood(logger);
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        Trajectory100 trajectory = ex.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, trajectory.sample(0).velocityM_S(), DELTA);
        ControllerSE2 controller = ControllerFactorySE2.test(logger);

        Gyro gyro = new SimulatedGyro(logger, swerveKinodynamics, collection);
        SwerveHistory history = new SwerveHistory(
                logger,
                swerveKinodynamics,
                Rotation2d.kZero,
                SwerveModulePositions.kZero(),
                Pose2d.kZero,
                0); // initial time is zero here for testing
        OdometryUpdater odometryUpdater = new OdometryUpdater(swerveKinodynamics, gyro, history, collection::positions);
        odometryUpdater.reset(Pose2d.kZero, 0);

        NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(history, odometryUpdater);

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, fieldLogger, layout, history, visionUpdater);
        FreshSwerveEstimate estimate = new FreshSwerveEstimate(localizer, odometryUpdater, history);
        SwerveLocal swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);
        SwerveLimiter limiter = new SwerveLimiter(logger, swerveKinodynamics, () -> 12);

        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(
                logger,
                odometryUpdater,
                estimate,
                swerveLocal,
                limiter);

        // initially at rest
        assertEquals(0, collection.states().frontLeft().speedMetersPerSecond(), DELTA);
        assertEquals(0, collection.states().frontLeft().angle().get().getRadians(), DELTA);

        DriveWithTrajectory command = new DriveWithTrajectory(
                logger, drive, controller, trajectory, viz);
        stepTime();
        command.initialize();

        command.execute();
        // but that output is not available until after takt.
        assertEquals(0, collection.states().frontLeft().speedMetersPerSecond(), DELTA);
        assertEquals(0, collection.states().frontLeft().angle().get().getRadians(), DELTA);

        // drive normally more
        stepTime();
        command.execute();
        // this is the output from the previous takt
        assertEquals(0.033, collection.states().frontLeft().speedMetersPerSecond(), DELTA);
        assertEquals(0, collection.states().frontLeft().angle().get().getRadians(), DELTA);

        // etc
        stepTime();
        command.execute();
        assertEquals(0.077, collection.states().frontLeft().speedMetersPerSecond(), DELTA);
        assertEquals(0, collection.states().frontLeft().angle().get().getRadians(), DELTA);
    }

}
