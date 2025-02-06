package org.team100.frc2025.Swerve;


import org.team100.frc2025.Swerve.SemiAuto.DriveTo_AB;
import org.team100.frc2025.Swerve.SemiAuto.DriveTo_ABNew;
import org.team100.frc2025.Swerve.SemiAuto.DriveTo_CD;
import org.team100.frc2025.Swerve.SemiAuto.DriveTo_EF;
import org.team100.frc2025.Swerve.SemiAuto.DriveTo_GH;
import org.team100.frc2025.Swerve.SemiAuto.DriveTo_IJ;
import org.team100.frc2025.Swerve.SemiAuto.DriveTo_KL;
import org.team100.lib.follower.DrivePIDFFollower;
import org.team100.lib.follower.DrivePIDFLockFollower;
import org.team100.lib.follower.DriveTrajectoryFollowerFactory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;


public class Maker {

    TimingConstraintFactory constraints;
    DrivePIDFFollower.Log m_PIDFLog;
    DrivePIDFLockFollower.Log m_lockedLog;

    LoggerFactory m_logger;
    DriveTo_GH.Log m_driveToGHLog;
    DriveTo_IJ.Log m_driveToIJLog;
    DriveTo_KL.Log m_driveToKLLog;
    DriveTo_AB.Log m_driveToABLog;
    DriveTo_CD.Log m_driveToCDLog;
    DriveTo_EF.Log m_driveToEFLog;


    SwerveDriveSubsystem m_swerve;
    DriveTrajectoryFollowerFactory m_factory;
    TrajectoryVisualization m_viz;
    SwerveKinodynamics m_kinodynamics;

    Trajectory100 m_clockWiseTraj;

    public Maker(LoggerFactory parent, SwerveDriveSubsystem swerve,  DriveTrajectoryFollowerFactory factory, SwerveKinodynamics kinodynamics, TrajectoryVisualization viz){
        m_logger = parent.child("Maker");
        m_PIDFLog = new DrivePIDFFollower.Log(m_logger);
        m_lockedLog = new DrivePIDFLockFollower.Log(m_logger);

        m_driveToGHLog = new DriveTo_GH.Log(m_logger);
        m_driveToIJLog = new DriveTo_IJ.Log(m_logger);
        m_driveToKLLog = new DriveTo_KL.Log(m_logger);
        m_driveToABLog = new DriveTo_AB.Log(m_logger);
        m_driveToCDLog = new DriveTo_CD.Log(m_logger);
        m_driveToEFLog = new DriveTo_EF.Log(m_logger);

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
                m_factory.goodPIDF(m_PIDFLog),
                m_viz,
                m_kinodynamics);

    }

    

    
}
