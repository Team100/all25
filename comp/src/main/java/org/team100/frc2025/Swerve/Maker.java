package org.team100.frc2025.Swerve;

import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.Swerve.Auto.GoToCoralStationLeft;
import org.team100.frc2025.Swerve.Auto.GoToIJLeft;
import org.team100.frc2025.Swerve.Auto.GoToReefDestination;
import org.team100.frc2025.Swerve.SemiAuto.Hexagon_Nav.Generate120;
import org.team100.frc2025.Swerve.SemiAuto.Hexagon_Nav.Generate180;
import org.team100.frc2025.Swerve.SemiAuto.Hexagon_Nav.Generate60;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
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

    public Command test() {
        final SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();
        final SwerveController holonomicController = SwerveControllerFactory.ridiculous(m_logger);


        final HolonomicProfile profile = new HolonomicProfile(
                swerveKinodynamics.getMaxDriveVelocityM_S(),
                swerveKinodynamics.getMaxDriveAccelerationM_S2(),
                0.01, // 1 cm
                swerveKinodynamics.getMaxAngleSpeedRad_S(),
                swerveKinodynamics.getMaxAngleAccelRad_S2(),
                0.1); // 5 degrees


        return new SequentialCommandGroup(
            new GoToReefDestination(
                m_logger,
                m_drive,
                SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger),
                m_viz,
                m_kinodynamics,
                FieldSector.IJ,
                ReefDestination.RIGHT),
            new GoToCoralStationLeft(
                    m_logger,
                    m_drive,
                    SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger),
                    m_viz,
                    m_kinodynamics,
                    0.25),
            new GoToReefDestination(
                    m_logger,
                    m_drive,
                    SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger),
                    m_viz,
                    m_kinodynamics,
                    FieldSector.KL,
                    ReefDestination.LEFT),
            new GoToCoralStationLeft(
                    m_logger,
                    m_drive,
                    SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger),
                    m_viz,
                    m_kinodynamics,
                    0),
            new GoToReefDestination(
                    m_logger,
                    m_drive,
                    SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger),
                    m_viz,
                    m_kinodynamics,
                    FieldSector.KL,
                    ReefDestination.RIGHT)
        );

        // return new GoToCoralStationLeft(
        //     m_logger,
        //     m_drive,
        //     SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger),
        //     m_viz,
        //     m_kinodynamics,
        //     0.5);
        // return new Embark(m_drive, holonomicController, profile);

    }

}
