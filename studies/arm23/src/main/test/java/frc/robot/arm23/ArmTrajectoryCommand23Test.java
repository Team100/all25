package org.team100.lib.commands.arm23;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.arm23.ArmAngles23;
import org.team100.lib.motion.arm23.ArmFactory23;
import org.team100.lib.motion.arm23.ArmKinematics23;
import org.team100.lib.motion.arm23.ArmSubsystem23;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

class ArmTrajectoryCommand23Test implements Timeless {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSimple() {
        ArmSubsystem23 ArmSubsystem23 = ArmFactory23.get(logger);
        ArmKinematics23 ArmKinematics23M = new ArmKinematics23(1, 1);
        Translation2d goal = new Translation2d();
        ArmTrajectoryCommand23 command = new ArmTrajectoryCommand23(
                logger,
                ArmSubsystem23,
                ArmKinematics23M,
                goal);
        command.initialize();
        assertEquals(0, ArmSubsystem23.getPosition().get().th1(), kDelta);
        stepTime();
        command.execute();
        // the goal is impossible so this is always finished.
        assertTrue(command.isFinished());
        command.end(false);
        ArmSubsystem23.close();
    }

    @Test
    void testSimple2() {
        ArmSubsystem23 ArmSubsystem23 = ArmFactory23.get(logger);
        ArmKinematics23 ArmKinematics23M = new ArmKinematics23(1, 1);
        Translation2d goal = new Translation2d(1, 1);
        ArmTrajectoryCommand23 command = new ArmTrajectoryCommand23(
                logger,
                ArmSubsystem23,
                ArmKinematics23M,
                goal);
        command.initialize();
        for (int i = 0; i < 800; ++i) {
            stepTime();
            command.execute();
            assertFalse(command.isFinished());
        }
        // let the controllers catch up
        for (int i = 0; i < 30; ++i) {
            stepTime();
            command.execute();
        }
        assertTrue(command.isFinished());
        // command tolerance is 0.02
        assertEquals(0, ArmSubsystem23.getPosition().get().th1(), 0.02);
        assertEquals(Math.PI / 2, ArmSubsystem23.getPosition().get().th2(), 0.02);
        command.end(false);
        ArmSubsystem23.close();
    }

    @Test
    void testPosRefernce() {
        ArmSubsystem23 ArmSubsystem23 = ArmFactory23.get(logger);
        ArmKinematics23 ArmKinematics23M = new ArmKinematics23(1, 1);
        Translation2d goal = new Translation2d(1, 1);
        ArmTrajectoryCommand23 command = new ArmTrajectoryCommand23(
                logger,
                ArmSubsystem23,
                ArmKinematics23M,
                goal);
        Trajectory.State s = new Trajectory.State();
        s.poseMeters = new Pose2d(1, 1, Rotation2d.kZero);
        ArmAngles23 r = command.getThetaPosReference(s);
        assertEquals(0, r.th1(), kDelta);
        assertEquals(Math.PI / 2, r.th2(), kDelta);
    }

    @Test
    void testVelRefernce() {
        ArmSubsystem23 ArmSubsystem23 = ArmFactory23.get(logger);
        ArmKinematics23 ArmKinematics23M = new ArmKinematics23(1, 1);
        Translation2d goal = new Translation2d(1, 1);
        ArmTrajectoryCommand23 command = new ArmTrajectoryCommand23(
                logger,
                ArmSubsystem23,
                ArmKinematics23M,
                goal);
        Trajectory.State s = new Trajectory.State();
        // zero rotation means path straight up
        s.poseMeters = new Pose2d(1, 1, Rotation2d.kZero);
        s.velocityMetersPerSecond = 1;
        ArmAngles23 r = command.getThetaPosReference(s);
        // proximal straight up
        assertEquals(0, r.th1(), kDelta);
        // distal at +90
        assertEquals(Math.PI / 2, r.th2(), kDelta);
        ArmAngles23 rdot = command.getThetaVelReference(s, r);
        // proximal does not move
        assertEquals(0, rdot.th1(), kDelta);
        // distal should be moving negative
        assertEquals(-1, rdot.th2(), kDelta);
    }
}
