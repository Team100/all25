package org.team100.frc2025.Swerve;

import org.team100.frc2025.Swerve.SemiAuto.SemiAuto_i3.Generate120;
import org.team100.lib.follower.TrajectoryFollowerFactory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;

public class Maker {

    TimingConstraintFactory constraints;

    LoggerFactory m_logger;

    SwerveDriveSubsystem m_swerve;
    TrajectoryFollowerFactory m_factory;
    TrajectoryVisualization m_viz;
    SwerveKinodynamics m_kinodynamics;

    Trajectory100 m_clockWiseTraj;

    public Maker(
            LoggerFactory parent,
            SwerveDriveSubsystem swerve,
            SwerveKinodynamics kinodynamics,
            TrajectoryVisualization viz) {
        m_logger = parent.child("Maker");
        m_swerve = swerve;
        constraints = new TimingConstraintFactory(kinodynamics);
        m_viz = viz;
        m_kinodynamics = kinodynamics;

    }

    public Command test() {

        return new Generate120(
                m_logger,
                m_swerve,
                TrajectoryFollowerFactory.fieldRelativeGoodPIDF(m_logger),
                m_viz,
                m_kinodynamics);

    }

}
