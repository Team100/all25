package org.team100.frc2025.Swerve;

import org.team100.frc2025.FieldConstants.CoralStation;
import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.Swerve.Auto.GoToCoralStation;
import org.team100.frc2025.Swerve.Auto.GoToFirstPlace;
import org.team100.frc2025.Swerve.Auto.GoToReefDestination;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.lib.controller.drivetrain.FullStateSwerveController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Maker {

    TimingConstraintFactory constraints;

    LoggerFactory m_logger;

    SwerveDriveSubsystem m_drive;
    TrajectoryVisualization m_viz;
    SwerveKinodynamics m_kinodynamics;

    Trajectory100 m_clockWiseTraj;

    public Maker(
            LoggerFactory parent,
            SwerveDriveSubsystem swerve,
            SwerveKinodynamics kinodynamics,
            TrajectoryVisualization viz) {
        m_logger = parent.child("Maker");
        m_drive = swerve;
        constraints = new TimingConstraintFactory(kinodynamics);
        m_viz = viz;
        m_kinodynamics = kinodynamics;

    }

    public Command embark() {
        final SwerveController holonomicController = SwerveControllerFactory.ridiculous(m_logger);


        final HolonomicProfile profile = new HolonomicProfile(
                m_kinodynamics.getMaxDriveVelocityM_S(),
                m_kinodynamics.getMaxDriveAccelerationM_S2(),
                0.01, // 1 cm
                m_kinodynamics.getMaxAngleSpeedRad_S(),
                m_kinodynamics.getMaxAngleAccelRad_S2(),
                0.1); // 5 degrees
        return new Embark(m_drive, holonomicController, profile);
    }

    public Command test(FieldLogger.Log fieldLog,SwerveController controller, HolonomicProfile profile) {
    

    return new SequentialCommandGroup(
                new GoToReefDestination(
                    fieldLog,
                        m_logger,
                        m_drive,
                        controller,
                        m_viz,
                        m_kinodynamics,
                        FieldSector.IJ,
                        ReefDestination.RIGHT,
                        profile),
                new GoToCoralStation(
                        m_logger,
                        m_drive,
                        controller,
                        m_viz,
                        m_kinodynamics,
                        CoralStation.Left,
                        0.5),
                new GoToReefDestination(fieldLog,
                        m_logger,
                        m_drive,
                        controller,
                        m_viz,
                        m_kinodynamics,
                        FieldSector.KL,
                        ReefDestination.RIGHT,
                        profile),
                new GoToCoralStation(
                        m_logger,
                        m_drive,
                        controller,
                        m_viz,
                        m_kinodynamics,
                        CoralStation.Left,
                        0),
                new GoToReefDestination(fieldLog,
                        m_logger,
                        m_drive,
                        controller,
                        m_viz,
                        m_kinodynamics,
                        FieldSector.KL,
                        ReefDestination.LEFT,
                        profile));

    // }

    // return new SequentialCommandGroup(
    //             new GoToDestinationDirectly(
    //                     m_logger,
    //                     m_drive,
    //                     SwerveControllerFactory.autoFieldRelativePIDF(m_logger),
    //                     m_viz,
    //                     m_kinodynamics,
    //                     FieldSector.EF,
    //                     ReefDestination.RIGHT),
    //             new GoToCoralStation(
    //                     m_logger,
    //                     m_drive,
    //                     SwerveControllerFactory.autoFieldRelativePIDF(m_logger),
    //                     m_viz,
    //                     m_kinodynamics,
    //                     CoralStation.Right,
    //                     0.5),
    //             new GoToDestinationDirectly(
    //                     m_logger,
    //                     m_drive,
    //                     SwerveControllerFactory.autoFieldRelativePIDF(m_logger),
    //                     m_viz,
    //                     m_kinodynamics,
    //                     FieldSector.CD,
    //                     ReefDestination.RIGHT),
    //             new GoToCoralStation(
    //                     m_logger,
    //                     m_drive,
    //                     SwerveControllerFactory.autoFieldRelativePIDF(m_logger),
    //                     m_viz,
    //                     m_kinodynamics,
    //                     CoralStation.Right,
    //                     0),
    //             new GoToDestinationDirectly(
    //                     m_logger,
    //                     m_drive,
    //                     SwerveControllerFactory.autoFieldRelativePIDF(m_logger),
    //                     m_viz,
    //                     m_kinodynamics,
    //                     FieldSector.CD,
    //                     ReefDestination.LEFT));

    }

    // return new SequentialCommandGroup(
    //             new GoToFirstPlace(
    //                     m_logger,
    //                     m_drive,
    //                     SwerveControllerFactory.autoFieldRelativePIDF(m_logger),
    //                     m_viz,
    //                     m_kinodynamics,
    //                     FieldSector.EF,
    //                     ReefDestination.RIGHT));
                //         ,
                // new GoToCoralStation(
                //         m_logger,
                //         m_drive,
                //         SwerveControllerFactory.autoFieldRelativePIDF(m_logger),
                //         m_viz,
                //         m_kinodynamics,
                //         CoralStation.Right,
                //         0.5),
                // new GoToDestinationDirectly(
                //         m_logger,
                //         m_drive,
                //         SwerveControllerFactory.autoFieldRelativePIDF(m_logger),
                //         m_viz,
                //         m_kinodynamics,
                //         FieldSector.CD,
                //         ReefDestination.RIGHT),
                // new GoToCoralStation(
                //         m_logger,
                //         m_drive,
                //         SwerveControllerFactory.autoFieldRelativePIDF(m_logger),
                //         m_viz,
                //         m_kinodynamics,
                //         CoralStation.Right,
                //         0),
                // new GoToDestinationDirectly(
                //         m_logger,
                //         m_drive,
                //         SwerveControllerFactory.autoFieldRelativePIDF(m_logger),
                //         m_viz,
                //         m_kinodynamics,
                //         FieldSector.CD,
                //         ReefDestination.LEFT));

    }