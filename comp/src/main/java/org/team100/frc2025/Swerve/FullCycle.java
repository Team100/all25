package org.team100.frc2025.Swerve;

import java.util.List;

import org.team100.frc2025.FieldConstants;
import org.team100.lib.commands.drivetrain.DriveToWaypoint3;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.ConstantConstraint;
import org.team100.lib.trajectory.TrajectoryMaker;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * An example of a "full auto" strategy, running a sequence of commands. Put
 * this inside a RepeatCommand to run it continuously.
 */
public class FullCycle extends SequentialCommandGroup implements Glassy {
    private static final double maxVelocityM_S = 2.0;
    private static final double maxAccelM_S_S = 2;
    private static final Pose2d waypoint0 = new Pose2d(6, 2, GeometryUtil.kRotationZero);
    private static final Pose2d waypoint1 = new Pose2d(2, 2, GeometryUtil.kRotationZero);

    public FullCycle(
            LoggerFactory parent,
            SwerveDriveSubsystem drivetrain,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics,
            SwerveController controller) {
        DriveToWaypoint3.Log log = new DriveToWaypoint3.Log(parent.child(this));
        TrajectoryMaker tmaker = new TrajectoryMaker(List.of(new ConstantConstraint(maxVelocityM_S, maxAccelM_S_S)));

        // StraightLineTrajectory maker = new StraightLineTrajectory(true, tmaker);
        Maker makerTrajec = new Maker(parent, drivetrain, kinodynamics, viz);
        Translation2d reefCenter = FieldConstants.getReefCenter();

        // for now just drive back and forth.
        addCommands(
                // new ResetPose(drivetrain, 6.305274, 5.979709, 0),

                // new RepeatCommand(
                makerTrajec.test()
        // )

        // new DriveToPoseSimple(parent, controller , drivetrain, makerTrajec)
        // makerTrajec.test(drivetrain::getPose)
        );

    }

}
