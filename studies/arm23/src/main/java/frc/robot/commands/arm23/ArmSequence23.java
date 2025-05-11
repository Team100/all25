package org.team100.lib.commands.arm23;

import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.arm23.ArmKinematics23;
import org.team100.lib.motion.arm23.ArmSubsystem23;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Sequence of arm trajectories.
 * 
 * These paint a square in cartesian space.
 */
public class ArmSequence23 extends SequentialCommandGroup100 {

    public ArmSequence23(LoggerFactory logger, ArmSubsystem23 armSubsystem, ArmKinematics23 armKinematicsM) {
        super(logger, "Sequence");
        addCommands(new ArmTrajectoryCommand23(logger, armSubsystem, armKinematicsM, new Translation2d(.6, .6)),
                new ArmTrajectoryCommand23(logger, armSubsystem, armKinematicsM, new Translation2d(1, .6)),
                new ArmTrajectoryCommand23(logger, armSubsystem, armKinematicsM, new Translation2d(1, 1)),
                new ArmTrajectoryCommand23(logger, armSubsystem, armKinematicsM, new Translation2d(.6, 1)),
                new ArmTrajectoryCommand23(logger, armSubsystem, armKinematicsM, new Translation2d(.6, .6)));
    }
}
