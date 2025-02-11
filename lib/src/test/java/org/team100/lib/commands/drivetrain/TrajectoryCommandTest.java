package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.drivetrain.HolonomicDriveControllerFactory;
import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.MockDrive;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryMaker;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;

public class TrajectoryCommandTest extends Fixtured implements Timeless {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TrajectoryVisualization viz = new TrajectoryVisualization(logger);
    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();
    List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
    TrajectoryMaker maker = new TrajectoryMaker(constraints);

    @Test
    void testTrajectoryStart() {
        Trajectory100 t = maker.restToRest(
                new Pose2d(0, 0, GeometryUtil.kRotationZero),
                new Pose2d(1, 0, GeometryUtil.kRotationZero));
        // first state is motionless
        assertEquals(0, t.sample(0).velocityM_S(), kDelta);
        HolonomicFieldRelativeController controller = HolonomicDriveControllerFactory.get(
                logger,
                new HolonomicFieldRelativeController.Log(logger));

        MockDrive d = new MockDrive();
        // initially at rest
        d.m_state = new SwerveModel();
        d.m_aligned = false;

        TrajectoryCommand c = new TrajectoryCommand(
                logger, d, controller, t, viz);
        c.initialize();

        // steering at rest
        c.execute();
        assertEquals(0.098, d.m_atRestSetpoint.x(), kDelta);
        assertEquals(0, d.m_atRestSetpoint.y(), kDelta);
        assertEquals(0, d.m_atRestSetpoint.theta(), kDelta);

        // we don't advance because we're still steering.
        // this next-setpoint is from "preview"
        // and our current setpoint is equal to the measurement.
        c.execute();
        assertEquals(0.098, d.m_atRestSetpoint.x(), kDelta);
        assertEquals(0, d.m_atRestSetpoint.y(), kDelta);
        assertEquals(0, d.m_atRestSetpoint.theta(), kDelta);

        d.m_aligned = true;
        // this time we return true but we still haven't advanced.
        // this next-setpoint is from "preview"
        // and our current setpoint is equal to the measurement.
        c.execute();
        assertEquals(0.098, d.m_atRestSetpoint.x(), kDelta);
        assertEquals(0, d.m_atRestSetpoint.y(), kDelta);
        assertEquals(0, d.m_atRestSetpoint.theta(), kDelta);

        // now we advance
        // but it's the same as the "previewed" setpoint above
        // and our current setpoint is still equal to the measurement.
        c.execute();
        assertEquals(0.098, d.m_setpoint.x(), kDelta);
        assertEquals(0, d.m_setpoint.y(), kDelta);
        assertEquals(0, d.m_setpoint.theta(), kDelta);

        // now we advance
        // finally with the next setpoint
        // and now our current setpoint is ahead of the measurement
        c.execute();
        assertEquals(0.199, d.m_setpoint.x(), kDelta);
        assertEquals(0, d.m_setpoint.y(), kDelta);
        assertEquals(0, d.m_setpoint.theta(), kDelta);
    }

    @Test
    void testTrajectoryDone() {
        Trajectory100 t = maker.restToRest(
                new Pose2d(0, 0, GeometryUtil.kRotationZero),
                new Pose2d(1, 0, GeometryUtil.kRotationZero));
        // first state is motionless
        assertEquals(0, t.sample(0).velocityM_S(), kDelta);
        HolonomicFieldRelativeController controller = HolonomicDriveControllerFactory.get(
                logger,
                new HolonomicFieldRelativeController.Log(logger));

        MockDrive d = new MockDrive();
        // initially at rest
        d.m_state = new SwerveModel();
        // for this test we don't care about steering alignment.
        d.m_aligned = true;

        TrajectoryCommand c = new TrajectoryCommand(
                logger, d, controller, t, viz);
        c.initialize();

        // the measurement never changes but that doesn't affect "done" as far as the
        // trajectory is concerned.
        for (int i = 0; i < 48; ++i) {
            c.execute();
        }
        assertTrue(c.isFinished());

    }

    /** Use a real drivetrain to observe the effect on the motors etc. */
    @Test
    void testRealDrive() {
        // 1m along +x, no rotation.
        Trajectory100 trajectory = maker.restToRest(
                new Pose2d(0, 0, GeometryUtil.kRotationZero),
                new Pose2d(1, 0, GeometryUtil.kRotationZero));
        // first state is motionless
        assertEquals(0, trajectory.sample(0).velocityM_S(), kDelta);
        HolonomicFieldRelativeController controller = HolonomicDriveControllerFactory.get(
                logger,
                new HolonomicFieldRelativeController.Log(logger));

        SwerveDriveSubsystem drive = fixture.drive;

        // initially at rest
        assertEquals(0, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);

        // initial state is wheels pointing +x
        assertTrue(drive.aligned(new FieldRelativeVelocity(1, 0, 0)));

        TrajectoryCommand command = new TrajectoryCommand(
                logger, drive, controller, trajectory, viz);
                stepTime(0.02);
        command.initialize();

        // always start unaligned
        assertFalse(command.m_aligned);

        // steering at rest
        // the old method takes a cycle of doing nothing.
        // it "previews" the next step (which is moving slowly +x) and steers there even
        // if we're already there.
        // ideally this would not happen, we would advance right away.
        command.execute();
        assertEquals(0, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);

        // the side-effect is to set the "aligned" flag.
        assertTrue(command.m_aligned);
        // and we are actually aligned (as we have been the whole time)
        assertTrue(drive.aligned(new FieldRelativeVelocity(1, 0, 0)));

        // actuate advance
        stepTime(0.02);
        command.execute();
        assertEquals(0, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);

        // observe actuation in measurements
        stepTime(0.02);
        command.execute();
        assertEquals(0.02, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);
    }

    /** Use a real drivetrain to observe the effect on the motors etc. */
    @Test
    void testRealDriveUnaligned() {
        // 1m along +y, no rotation.
        Trajectory100 trajectory = maker.restToRest(
                new Pose2d(0, 0, GeometryUtil.kRotationZero),
                new Pose2d(0, 1, GeometryUtil.kRotationZero));
        // first state is motionless
        assertEquals(0, trajectory.sample(0).velocityM_S(), kDelta);
        HolonomicFieldRelativeController controller = HolonomicDriveControllerFactory.get(
                logger,
                new HolonomicFieldRelativeController.Log(logger));

        SwerveDriveSubsystem drive = fixture.drive;

        // initially at rest
        assertEquals(0, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);
        // d.m_state = new SwerveModel();
        // d.m_aligned = false;
        // initial state is wheels pointing +x
        assertTrue(drive.aligned(new FieldRelativeVelocity(1, 0, 0)));

        TrajectoryCommand command = new TrajectoryCommand(
                logger, drive, controller, trajectory, viz);
        command.initialize();
        // always start unaligned
        assertFalse(command.m_aligned);

        // steering at rest for awhile, for the wheels to turn 90 degrees.
        // this is 16 cycles of 20 ms which is 0.32 sec, a long time. the profile
        // here is acceleration-limited, 20pi/s^2, which is just what the limits say.
        // https://docs.google.com/spreadsheets/d/19N-rQBjlWM-fb_Hd5wJeKuihzoglmGVK7MT7RYU-9bY
        for (int i = 0; i < 17; ++i) {
            // Util.printf("************ %d **********", i);
            // command thinks we're not aligned
            assertFalse(command.m_aligned);
            // drive thinks we're not aligned to the target (0,1)
            assertFalse(drive.aligned(new FieldRelativeVelocity(0, 1, 0)));

            stepTime(0.02);
            fixture.drive.periodic();
            // this calls steerAtRest, using the previewed state.
            command.execute();
            // assertEquals(0,
            // fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
            // assertEquals(0,
            // fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);
            // Util.printf("angle %s\n", fixture.collection.states().frontLeft().angle().get());
            // Util.printf("p %s\n", fixture.collection.turningPosition()[0].getAsDouble());
            // Util.printf("v %s\n", fixture.collection.turningVelocity()[0].getAsDouble());
        }
        // the last call to steer at rest discovers that the modules are already at the goal
        //
        // Util.println("aligned?");
        assertTrue(command.m_aligned);
        assertTrue(drive.aligned(new FieldRelativeVelocity(0, 1, 0)));
        // 
        // note the steering tolerance means we're not *exactly* in the right place.
        assertEquals(0, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        // This overshoots slightly due to the clock granularity
        assertEquals(1.573, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(1.573, fixture.collection.turningPosition()[0].getAsDouble(), kDelta);
        assertEquals(0, fixture.collection.turningVelocity()[0].getAsDouble(), kDelta);

        // now we advance in +y.

        // increment the clock
        stepTime(0.02);

        assertTrue(drive.aligned(new FieldRelativeVelocity(0, 1, 0)));
        assertTrue(command.m_aligned);
        assertEquals(1.572, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(1.572, fixture.collection.turningPosition()[0].getAsDouble(), kDelta);
        assertEquals(-0.039, fixture.collection.turningVelocity()[0].getAsDouble(), kDelta);

        command.execute();

        assertTrue(drive.aligned(new FieldRelativeVelocity(0, 1, 0)));
        assertTrue(command.m_aligned);

        // motor speed command has not changed the velocity yet, due to the takt cache.
        assertEquals(0, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(1.572, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(1.572, fixture.collection.turningPosition()[0].getAsDouble(), kDelta);
        assertEquals(-0.039, fixture.collection.turningVelocity()[0].getAsDouble(), kDelta);

        // now the velocity measurement reflects the previous actuation
        stepTime(0.02);
        command.execute();
        assertEquals(0.02, fixture.collection.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(1.572, fixture.collection.states().frontLeft().angle().get().getRadians(), kDelta);

    }

}
