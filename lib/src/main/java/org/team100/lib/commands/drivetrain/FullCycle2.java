package org.team100.lib.commands.drivetrain;

import java.util.List;

import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.ConstantConstraint;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * An example of a "full auto" strategy, running a sequence of commands. Put
 * this inside a RepeatCommand to run it continuously.
 */
public class FullCycle2 extends SequentialCommandGroup100 implements Glassy {
    // private static final double maxVelocityM_S = 2.0;
    // private static final double maxAccelM_S_S = 2;
    // private static final Pose2d waypoint0 = new Pose2d(6, 2, Rotation2d.kZero);
    // private static final Pose2d waypoint1 = new Pose2d(2, 2, Rotation2d.kZero);

    public FullCycle2(
            LoggerFactory parent,
            SwerveDriveSubsystem drive,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics,
            SwerveController controller) {
        super(parent, "FullCycle2");
        TrajectoryPlanner planner = new TrajectoryPlanner(
                List.of(new ConstantConstraint(1, 1, kinodynamics)));

        // StraightLineTrajectory maker = new StraightLineTrajectory(true, tmaker);
        // Maker makerTrajec = new Maker(parent, drivetrain, factory, kinodynamics,
        // viz);
        // Translation2d reefCenter = FieldConstants.getReefCenter();

        // for now just drive back and forth.
        addCommands(
                new ResetPose(drive, new Pose2d(1.86, 6.6, new Rotation2d(0)))

        // new RepeatCommand(
        // // makerTrajec.makeTrajectoryCommand()
        // )

        // new DriveToPoseSimple(parent, controller , drivetrain, makerTrajec)
        // makerTrajec.runClockWiseTraj()
        );

    }

}
