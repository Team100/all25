package org.team100.frc2025.Swerve.SemiAuto.Hexagon_Nav;

import org.team100.lib.commands.drivetrain.FieldConstants;
import org.team100.lib.commands.drivetrain.ReefPath;
import org.team100.lib.commands.drivetrain.FieldConstants.FieldSector;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefDestination;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class Picker {

    TimingConstraintFactory constraints;

    LoggerFactory m_logger;

    SwerveDriveSubsystem m_drive;
    TrajectoryVisualization m_viz;
    SwerveKinodynamics m_kinodynamics;

    public Picker(
            LoggerFactory parent,
            SwerveDriveSubsystem swerve,
            SwerveKinodynamics kinodynamics,
            TrajectoryVisualization viz) {

        m_logger = parent.child("Picker");
        m_drive = swerve;
        constraints = new TimingConstraintFactory(kinodynamics);
        m_viz = viz;
        m_kinodynamics = kinodynamics;

    }

    public Command pickReefPath(FieldConstants.FieldSector end, ReefDestination reefDestination) {
        Pose2d currentPose = m_drive.getPose();
        FieldSector start = FieldConstants.getSector(currentPose);

        ReefPath path = FieldConstants.findShortestPath(start.getValue(), end.getValue());

        switch (path.paths().size()) {
            case 4:
                return new Generate180(m_logger, m_drive, SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger),
                        m_viz, m_kinodynamics, end, reefDestination);
            case 3:
                return new Generate120(m_logger, m_drive, SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger),
                        m_viz, m_kinodynamics, end, reefDestination);
            case 2:
                return new Generate60(m_logger, m_drive, SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger), m_viz,
                        m_kinodynamics, end, reefDestination);
            case 1:
                return new Generate60(m_logger, m_drive, SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger), m_viz,
                        m_kinodynamics, end, reefDestination);
            default:
                return new Generate60(m_logger, m_drive, SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger), m_viz,
                        m_kinodynamics, end, reefDestination);

        }

    }
}
