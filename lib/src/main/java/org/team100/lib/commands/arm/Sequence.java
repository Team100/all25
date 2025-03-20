package org.team100.lib.commands.arm;

import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motion.arm.ArmSubsystem;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Sequence of arm trajectories.
 * 
 * These paint a square in cartesian space.
 */
public class Sequence extends SequentialCommandGroup100 {

    public Sequence(LoggerFactory parent, ArmSubsystem armSubsystem, ArmKinematics armKinematicsM) {
        super(parent);
        addCommands(new ArmTrajectoryCommand(parent, armSubsystem, armKinematicsM, new Translation2d(.6, .6)),
                new ArmTrajectoryCommand(parent, armSubsystem, armKinematicsM, new Translation2d(1, .6)),
                new ArmTrajectoryCommand(parent, armSubsystem, armKinematicsM, new Translation2d(1, 1)),
                new ArmTrajectoryCommand(parent, armSubsystem, armKinematicsM, new Translation2d(.6, 1)),
                new ArmTrajectoryCommand(parent, armSubsystem, armKinematicsM, new Translation2d(.6, .6)));
    }
}
