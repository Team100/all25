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
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.trajectory.PoseSet;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Generate120 extends Navigator {

    private final double kTangentScale = 1;
    private final double kEntranceCurveFactor = 0.25;

    public Generate120(
            LoggerFactory parent,
            SwerveDriveSubsystem drive,
            SwerveController hcontroller,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics) {
        super(parent, drive, hcontroller, viz, kinodynamics);
    }

    @Override
    public Trajectory100 trajectory(List<TimingConstraint> constraints, Pose2d currentPose) {

        Translation2d currTranslation = currentPose.getTranslation();

        FieldSector start = FieldConstants.getSector(currentPose);
        FieldSector end = FieldSector.AB;
        ReefDestination reefDestination = ReefDestination.CENTER;

        Translation2d destination = FieldConstants.getOrbitDestination(end, reefDestination);

        ReefPath path = FieldConstants.findShortestPath(start.getValue(), end.getValue());
        List<Integer> list = path.paths();
        ReefAproach approach = path.approach();

        FieldSector anchorPreviousSector = FieldSector.fromValue(list.get(0));
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

        return TrajectoryPlanner.restToRest(poseSet.poses(), poseSet.headings(), constraints);
    }

}
