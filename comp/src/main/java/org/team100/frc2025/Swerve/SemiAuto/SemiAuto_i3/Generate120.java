// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Swerve.SemiAuto.SemiAuto_i3;

import java.util.ArrayList;
import java.util.List;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefAproach;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.Swerve.SemiAuto.LandingDestinationGroup;
import org.team100.frc2025.Swerve.SemiAuto.Navigator;
import org.team100.frc2025.Swerve.SemiAuto.ReefPath;
import org.team100.lib.follower.TrajectoryFollower;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.PoseSet;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Generate120 extends Navigator {
  /** Creates a new Generate180. */

  private final double kTangentScale = 1;
  private final double kEntranceCurveFactor = 0.25;

  private final SwerveDriveSubsystem m_robotDrive;
  private final TrajectoryFollower m_controller;
  private Pose2d m_goal = new Pose2d();
  private final TrajectoryVisualization m_viz;
  TimingConstraintFactory m_constraints;

  public Generate120(LoggerFactory parent,
            SwerveDriveSubsystem robotDrive,
            TrajectoryFollower controller,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(parent, robotDrive, controller, viz, kinodynamics);
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
    Translation2d currTranslation = currPose.getTranslation();

    FieldSector start = FieldConstants.getSector(m_robotDrive.getPose());
    FieldSector end = FieldSector.AB;
    ReefDestination reefDestination = ReefDestination.CENTER;
    
    Translation2d destination = FieldConstants.getOrbitDestination(end, reefDestination);

    ReefPath path = FieldConstants.findShortestPath(start.getValue(), end.getValue());
    List<Integer> list = path.paths();
    ReefAproach approach = path.approach();


    FieldSector anchorPreviousSector = FieldSector.fromValue(list.get(0));
    Rotation2d anchorPreviousRotation = FieldConstants.getSectorAngle(anchorPreviousSector);

    Rotation2d anchorPointRotation = FieldConstants.calculateAnchorPointDelta(anchorPreviousRotation, approach);
    Translation2d anchorWaypoint =  FieldConstants.getOrbitWaypoint(anchorPointRotation);
    
    Translation2d landingZone = FieldConstants.getOrbitLandingZone(end, approach);


    List<Pose2d> waypointsM = new ArrayList<>();
    List<Rotation2d> headings = new ArrayList<>();

    LandingDestinationGroup rotationGroup = FieldConstants.getRotationGroup(approach, end, kEntranceCurveFactor);


    Rotation2d initialSpline = calculateInitialSpline(
        anchorWaypoint,
        currTranslation,
        currTranslation.minus(FieldConstants.getReefCenter()), //vector from robot to reef center
        approach,
        kTangentScale);

    double distance = 0.5; // Distance to move
    double newX = currTranslation.getX() + (distance * initialSpline.getCos());
    double newY = currTranslation.getY() + (distance * initialSpline.getSin());

    Translation2d newTranslation = new Translation2d(newX, newY);
    Pose2d newPose = new Pose2d(newTranslation, initialSpline);
    Rotation2d rotationToReef = FieldConstants.angleToReefCenter(newPose);


    waypointsM.add(newPose);
    waypointsM.add( new Pose2d(landingZone, rotationGroup.landingSpline()));
    waypointsM.add( new Pose2d(destination, rotationGroup.destinationSpline()));

    headings.add(rotationToReef);
    headings.add(FieldConstants.angleToReefCenter(new Pose2d(landingZone, rotationGroup.landingSpline())));
    headings.add(FieldConstants.angleToReefCenter( new Pose2d(destination, rotationGroup.destinationSpline())));


    
    
  

    
    PoseSet poseSet = addRobotPose(currPose, waypointsM, headings, initialSpline);

     Trajectory100 trajectory = TrajectoryPlanner.restToRest(poseSet.poses(), poseSet.headings(),
                m_constraints.medium());
    m_viz.setViz(trajectory);
    m_controller.setTrajectory(trajectory);

  }

}
