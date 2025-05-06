package org.team100.lib.commands.arm;

import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.arm23.ArmKinematics;
import org.team100.lib.motion.arm23.ArmSubsystem;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Sequence of arm trajectories.
 * 
 * These paint a square in cartesian space.
 */
public class Sequence extends SequentialCommandGroup100 {

    public Sequence(LoggerFactory logger, ArmSubsystem armSubsystem, ArmKinematics armKinematicsM) {
        super(logger, "Sequence");
        addCommands(new ArmTrajectoryCommand(logger, armSubsystem, armKinematicsM, new Translation2d(.6, .6)),
                new ArmTrajectoryCommand(logger, armSubsystem, armKinematicsM, new Translation2d(1, .6)),
                new ArmTrajectoryCommand(logger, armSubsystem, armKinematicsM, new Translation2d(1, 1)),
                new ArmTrajectoryCommand(logger, armSubsystem, armKinematicsM, new Translation2d(.6, 1)),
                new ArmTrajectoryCommand(logger, armSubsystem, armKinematicsM, new Translation2d(.6, .6)));
    }
}
