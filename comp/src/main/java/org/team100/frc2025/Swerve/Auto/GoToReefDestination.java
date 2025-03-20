package org.team100.frc2025.Swerve.Auto;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.lib.commands.drivetrain.DriveToPoseWithProfile;
import org.team100.lib.controller.drivetrain.FullStateSwerveController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GoToReefDestination extends SequentialCommandGroup {
    public GoToReefDestination(
            FieldLogger.Log fieldLog,
            LoggerFactory parent,
            SwerveDriveSubsystem drive,
            SwerveController hcontroller,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics,
            FieldSector endSector,
            ReefDestination reefDest,
            HolonomicProfile profile) {

        Translation2d destination = FieldConstants.getOrbitDestination(endSector, reefDest, 1.3);
        Rotation2d heading = FieldConstants.getSectorAngle(endSector).rotateBy(Rotation2d.fromDegrees(180));
        Pose2d m_goal = new Pose2d(destination, heading);
        SwerveController tet = new FullStateSwerveController(parent, 3, 3.5, 0.05, 0, 1, 1, 1, 1);
        addCommands(
                new GoToReefBuffer(parent, drive, tet, viz, kinodynamics, endSector, reefDest),
                new DriveToPoseWithProfile(fieldLog, () -> new SwerveModel(m_goal), drive, hcontroller, profile)

        );
    }
}
