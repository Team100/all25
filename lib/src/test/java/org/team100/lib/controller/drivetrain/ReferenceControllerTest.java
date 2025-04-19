package org.team100.lib.controller.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.MockDrive;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.testing.Timeless;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ReferenceControllerTest extends Fixtured implements Timeless {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
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

        MockDrive drive = new MockDrive();
        // initially at rest
        drive.m_state = new SwerveModel();

        ReferenceController c = new ReferenceController(
                drive,
                controller,
                new TrajectoryReference(t), false);

        stepTime();
        c.execute();

        // we don't advance because we're still steering.
        // this next-setpoint is from "preview"
        // and our current setpoint is equal to the measurement.
        stepTime();
        c.execute();
        // assertEquals(0.098, drive.m_atRestSetpoint.x(), kDelta);
        // assertEquals(0, drive.m_atRestSetpoint.y(), kDelta);
        // assertEquals(0, drive.m_atRestSetpoint.theta(), kDelta);

        stepTime();
        c.execute();
        assertEquals(0.139, drive.m_setpoint.x(), kDelta);
        assertEquals(0, drive.m_setpoint.y(), kDelta);
        assertEquals(0, drive.m_setpoint.theta(), kDelta);

        // more normal driving
        stepTime();
        c.execute();
        assertEquals(0.179, drive.m_setpoint.x(), kDelta);
        assertEquals(0, drive.m_setpoint.y(), kDelta);
        assertEquals(0, drive.m_setpoint.theta(), kDelta);

        // etc
        stepTime();
        c.execute();
        assertEquals(0.221, drive.m_setpoint.x(), kDelta);
        assertEquals(0, drive.m_setpoint.y(), kDelta);
        assertEquals(0, drive.m_setpoint.theta(), kDelta);
    }

    @Test
    void testTrajectoryDone() {
        Trajectory100 t = planner.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, t.sample(0).velocityM_S(), kDelta);
        SwerveController controller = SwerveControllerFactory.test(logger);

        MockDrive drive = new MockDrive();
        // initially at rest
        drive.m_state = new SwerveModel();

        ReferenceController c = new ReferenceController(
                drive,
                controller,
                new TrajectoryReference(t),
                false);

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

        ReferenceController command = new ReferenceController(
                drive,
                controller,
                new TrajectoryReference(trajectory),
                false);
        stepTime();

        command.execute();
        // but that output is not available until after takt.
        assertEquals(0, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);

        // drive normally more
        stepTime();
        command.execute();
        // this is the output from the previous takt
        // TODO: fix this test
        // assertEquals(0.02, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);

        // etc
        stepTime();
        command.execute();
        // assertEquals(0.04, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);
    }


}
