package org.team100.frc2025.Swerve;

import org.team100.frc2025.Swerve.SemiAuto.DriveTo_AB;
import org.team100.frc2025.Swerve.SemiAuto.DriveTo_ABNew;
import org.team100.frc2025.Swerve.SemiAuto.DriveTo_CD;
import org.team100.frc2025.Swerve.SemiAuto.DriveTo_EF;
import org.team100.frc2025.Swerve.SemiAuto.DriveTo_GH;
import org.team100.frc2025.Swerve.SemiAuto.DriveTo_IJ;
import org.team100.frc2025.Swerve.SemiAuto.DriveTo_KL;
import org.team100.lib.follower.DriveTrajectoryFollowerFactory;
import org.team100.lib.follower.FieldRelativeDrivePIDFFollower;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;

public class Maker {

    TimingConstraintFactory constraints;
    FieldRelativeDrivePIDFFollower.Log m_PIDFLog;

    LoggerFactory m_logger;

    SwerveDriveSubsystem m_swerve;
    DriveTrajectoryFollowerFactory m_factory;
    TrajectoryVisualization m_viz;
    SwerveKinodynamics m_kinodynamics;

    Trajectory100 m_clockWiseTraj;

    public Maker(
            LoggerFactory parent,
            SwerveDriveSubsystem swerve,
            DriveTrajectoryFollowerFactory factory,
            SwerveKinodynamics kinodynamics,
            TrajectoryVisualization viz) {
        m_logger = parent.child("Maker");
        m_PIDFLog = new FieldRelativeDrivePIDFFollower.Log(m_logger);
        m_factory = factory;
        m_swerve = swerve;
        constraints = new TimingConstraintFactory(kinodynamics);
        m_viz = viz;
        m_kinodynamics = kinodynamics;

    }

    public Command test() {

        return new DriveTo_ABNew(
                m_logger,
                m_swerve,
                m_factory.fieldRelativeGoodPIDF(m_PIDFLog),
                m_viz,
                m_kinodynamics);

    }

}
