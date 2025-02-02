// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.Swerve.SemiAuto;

import java.util.ArrayList;
import java.util.List;

import org.team100.frc2024.FieldConstants;
import org.team100.frc2024.FieldConstants.ReefPoint;
import org.team100.lib.follower.DrivePIDFFollower;
import org.team100.lib.follower.DrivePIDFLockFollower;
import org.team100.lib.follower.DriveTrajectoryFollower;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.PoseSet;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveTo_ABNew extends Navigator implements Planner2025 {
    /** Creates a new TrajectoryCommandWithPose100. */
    private final SwerveDriveSubsystem m_robotDrive;
    private final DriveTrajectoryFollower m_controller;
    private Pose2d m_goal = new Pose2d();
    private final TrajectoryVisualization m_viz;
    private final Navigator.Log m_log;

    TimingConstraintFactory m_constraints;

    public DriveTo_ABNew(
            LoggerFactory parent,
            SwerveDriveSubsystem robotDrive,
            DriveTrajectoryFollower controller,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics) {
        super(parent, robotDrive, controller, viz, kinodynamics);
        m_log = super.m_log;
        m_robotDrive = robotDrive;
        m_controller = controller;
        m_viz = viz;
        m_constraints = new TimingConstraintFactory(kinodynamics);
        addRequirements(m_robotDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d currPose = m_robotDrive.getPose();
        FieldConstants.FieldSector originSector = FieldConstants.getSector(currPose);
        FieldConstants.FieldSector destinationSector = FieldConstants.FieldSector.AB;
        FieldConstants.ReefPoint destinationPoint = FieldConstants.ReefPoint.A;

        List<Pose2d> waypointsM = new ArrayList<>();
        ;
        List<Rotation2d> headings = new ArrayList<>();
        ;
        List<Double> mN = new ArrayList<>();
        ;

        Translation2d currTranslation = currPose.getTranslation();
        Rotation2d initialSpline = new Rotation2d();

        Translation2d vectorFromCenterToRobot = currTranslation.minus(FieldConstants.getReefCenter());

        switch (originSector) {
            case AB:
                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                Rotation2d.fromDegrees(90)));

                headings.add(Rotation2d.fromDegrees(0));

                initialSpline = calculateInitialSpline(
                    FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                    currTranslation
                    );

                waypointsM.clear();

                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                initialSpline));

                break;
            case CD:


                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                        Rotation2d.fromDegrees(90)));

                headings.add(Rotation2d.fromDegrees(0));

                initialSpline = calculateInitialSpline(
                        FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                        currTranslation
                        );
                break;
            case EF:

                waypointsM.add(new Pose2d(2.71, 4.04 - 1.5, Rotation2d.fromDegrees(110)));

                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                        Rotation2d.fromDegrees(70)));

                headings.add(Rotation2d.fromDegrees(0));
                headings.add(Rotation2d.fromDegrees(0));

                initialSpline = calculateInitialSpline(
                        FieldConstants.getOrbitWaypoint(Rotation2d.fromDegrees(-90)),
                        currTranslation,
                        vectorFromCenterToRobot,
                        Rotation2d.fromDegrees(-90),
                        0.1);
                break;
            case GH:
                waypointsM.add(new Pose2d(2.71, 4.04 + 1.5, Rotation2d.fromDegrees(-110)));

                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                        Rotation2d.fromDegrees(-70)));

                headings.add(Rotation2d.fromDegrees(0));
                headings.add(Rotation2d.fromDegrees(0));

                initialSpline = calculateInitialSpline(
                        FieldConstants.getOrbitWaypoint(Rotation2d.fromDegrees(90)),
                        currTranslation,
                        vectorFromCenterToRobot,
                        Rotation2d.fromDegrees(90),
                        0.5);

                break;
            case IJ:
                waypointsM.add(new Pose2d(2.71, 4.04 + 1.5, Rotation2d.fromDegrees(-110)));

                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                        Rotation2d.fromDegrees(-70)));

                headings.add(Rotation2d.fromDegrees(0));
                headings.add(Rotation2d.fromDegrees(0));

                initialSpline = calculateInitialSpline(
                        FieldConstants.getOrbitWaypoint(Rotation2d.fromDegrees(90)),
                        currTranslation,
                        vectorFromCenterToRobot,
                        Rotation2d.fromDegrees(90),
                        0.1);
                break;
            case KL:

                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                        Rotation2d.fromDegrees(-90)));

                headings.add(Rotation2d.fromDegrees(0));

                initialSpline = calculateInitialSpline(
                        FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                        currTranslation
                        );
                break;
            default:
                break;

        }

        m_goal = waypointsM.get(waypointsM.size() - 1);

        PoseSet poseSet = addRobotPose(currPose, waypointsM, headings, initialSpline);

        Trajectory100 trajectory = TrajectoryPlanner.restToRest(poseSet.poses(), poseSet.headings(),
                m_constraints.fast());
        m_viz.setViz(trajectory);
        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(new TrajectoryTimeSampler(trajectory));
        m_controller.setTrajectory(iter);

    }

}
