package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.drivetrain.HolonomicDriveControllerFactory;
import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.MockDrive;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryMaker;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;

public class TrajectoryCommandTest  {
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
        assertEquals(0, t.getPoint(0).state().velocityM_S(), kDelta);
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
        assertEquals(0, t.getPoint(0).state().velocityM_S(), kDelta);
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

}
