package org.team100.frc2025.Swerve.SemiAuto.Hexagon_Nav;

import java.util.ArrayList;
import java.util.List;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefAproach;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.Swerve.SemiAuto.Navigator;
import org.team100.frc2025.Swerve.SemiAuto.ReefPath;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.PoseSet;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Generate180 extends Navigator {

    private final double kTangentScale = 3;
    private final double kEntranceCurveFactor = 0.25;

    private final FieldSector m_end;
    private final ReefDestination m_reefDestination;

    public Generate180(
            LoggerFactory parent,
            SwerveDriveSubsystem drive,
            SwerveController controller,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics,
            FieldSector endSector,
            ReefDestination reefDest) {
        super(parent, drive, controller, viz, kinodynamics);
        m_end = endSector;
        m_reefDestination = reefDest;
    }

    @Override
    public Trajectory100 trajectory(Pose2d currentPose) {

        Translation2d currTranslation = currentPose.getTranslation();

        FieldSector start = FieldConstants.getSector(currentPose);
        FieldSector end = m_end;
        ReefDestination reefDestination = m_reefDestination;

        Translation2d destination = FieldConstants.getOrbitDestination(end, reefDestination);

        ReefPath path = FieldConstants.findShortestPath(start.getValue(), end.getValue());
        List<Integer> list = path.paths();
        ReefAproach approach = path.approach();

        FieldSector anchorPreviousSector = FieldSector.fromValue(list.get(1));
        Rotation2d anchorPreviousRotation = FieldConstants.getSectorAngle(anchorPreviousSector);

        Rotation2d anchorPointRotation = FieldConstants.calculateAnchorPointDelta(anchorPreviousRotation, approach);
        Translation2d anchorWaypoint = FieldConstants.getOrbitWaypoint(anchorPointRotation);

        Translation2d landingZone = FieldConstants.getOrbitLandingZone(end, approach);

        List<Pose2d> waypointsM = new ArrayList<>();
        List<Rotation2d> headings = new ArrayList<>();

        LandingDestinationGroup rotationGroup = FieldConstants.getRotationGroup(approach, end, kEntranceCurveFactor);

        Rotation2d initialSpline = calculateInitialSpline(
                anchorWaypoint,
                currTranslation,
                currTranslation.minus(FieldConstants.getReefCenter()), // vector from robot to reef center
                approach,
                kTangentScale);

        double distance = 0.5; // Distance to move
        double newX = currTranslation.getX() + (distance * initialSpline.getCos());
        double newY = currTranslation.getY() + (distance * initialSpline.getSin());

        Translation2d newTranslation = new Translation2d(newX, newY);
        Pose2d newPose = new Pose2d(newTranslation, initialSpline);
        Rotation2d rotationToReef = FieldConstants.angleToReefCenter(newPose);

        waypointsM.add(newPose);
        waypointsM.add(new Pose2d(landingZone, rotationGroup.landingSpline()));
        waypointsM.add(new Pose2d(destination, rotationGroup.destinationSpline()));

        headings.add(rotationToReef);
        headings.add(FieldConstants.angleToReefCenter(new Pose2d(landingZone, rotationGroup.landingSpline())));
        headings.add(FieldConstants.angleToReefCenter(new Pose2d(destination, rotationGroup.destinationSpline())));

        PoseSet poseSet = addRobotPose(currentPose, waypointsM, headings, initialSpline);

        Translation2d currentSpeed = new Translation2d(m_drive.getChassisSpeeds().vxMetersPerSecond, m_drive.getChassisSpeeds().vyMetersPerSecond);
        // return m_planner.restToRest(poseSet.poses(), poseSet.headings(), constraints);
        return m_planner.generateTrajectory(poseSet.poses(), poseSet.headings(), currentSpeed.getNorm(), 0);

    }

}
