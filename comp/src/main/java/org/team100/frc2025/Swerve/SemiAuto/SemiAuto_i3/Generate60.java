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

public class Generate60 extends Navigator {

    private final double kTangentScale = 1;
    private final double kEntranceCurveFactor = 0.25;

    public Generate60(
            LoggerFactory parent,
            SwerveDriveSubsystem drive,
            SwerveController controller,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics) {
        super(parent, drive, controller, viz, kinodynamics);
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

        Translation2d landingZone = FieldConstants.getOrbitLandingZone(end, approach);

        List<Pose2d> waypointsM = new ArrayList<>();
        List<Rotation2d> headings = new ArrayList<>();

        LandingDestinationGroup rotationGroup = FieldConstants.getRotationGroup(approach, end, kEntranceCurveFactor);

        Rotation2d initialSpline = destination.minus(currTranslation).getAngle();

        Rotation2d initialSplineUnotuched = initialSpline;

        Rotation2d endSpline = new Rotation2d(0);

        // if(approach == ReefAproach.CCW){
        // Rotation2d newRotation =
        // parallelInitialRotation.plus(Rotation2d.fromDegrees(10));
        // condition = initialSpline.getDegrees() <= newRotation.getDegrees();
        // } else{
        // Rotation2d newRotation =
        // parallelInitialRotation.minus(Rotation2d.fromDegrees(10));
        // condition = initialSpline.getDegrees() >= newRotation.getDegrees();
        // }

        Rotation2d anchorPointRotation = FieldConstants.calculateAnchorPointDelta(FieldConstants.getSectorAngle(start),
                approach);
        Translation2d anchorWaypoint = FieldConstants.getOrbitWaypoint(anchorPointRotation);

        Translation2d pointToAnchor = anchorWaypoint.minus(FieldConstants.getReefCenter());
        // Translation2d perpVector = new Translation2d();

        // if(approach == ReefAproach.CCW){
        // perpVector = pointToAnchor.rotateBy(Rotation2d.fromDegrees(90));
        // }else{
        // perpVector = pointToAnchor.rotateBy(Rotation2d.fromDegrees(-90));
        // }

        Translation2d vectorToAnchorPoint = anchorWaypoint.minus(currTranslation);

        double dotProduct = (pointToAnchor.getX() * vectorToAnchorPoint.getX())
                + (pointToAnchor.getY() * vectorToAnchorPoint.getY());

        // System.out.println(dotProduct);

        // condition = rotationToAnchorPoint.getDegrees() <= 0;

        double distanceToReef = FieldConstants.getDistanceToReefCenter(currTranslation);

        boolean condition = false;
        Rotation2d parallelInitialRotation = FieldConstants.getLandingAngle(end, approach);


        if (dotProduct >= 0) {

            endSpline = FieldConstants.calculateDeltaSplineEnd(FieldConstants.getLandingAngle(end, approach),
                    FieldConstants.getSectorAngle(end), approach, 0.25);

            initialSpline = FieldConstants.calculateDeltaSpline(initialSpline, FieldConstants.getSectorAngle(start),
                    approach, (1 / (distanceToReef)));

        } else {
            endSpline = initialSpline;
        }

        double distance = 0.15; // Distance to move
        double newX = currTranslation.getX() + (distance * initialSpline.getCos());
        double newY = currTranslation.getY() + (distance * initialSpline.getSin());

        Translation2d newTranslation = new Translation2d(newX, newY);
        Pose2d newPose = new Pose2d(newTranslation, initialSpline);
        Rotation2d rotationToReef = FieldConstants.angleToReefCenter(newPose);

        waypointsM.add(newPose);
        waypointsM.add(new Pose2d(destination, endSpline));

        headings.add(rotationToReef);
        headings.add(FieldConstants.getSectorAngle(end).rotateBy(Rotation2d.k180deg));

        PoseSet poseSet = addRobotPose(currentPose, waypointsM, headings, initialSpline);

        return TrajectoryPlanner.restToRest(poseSet.poses(), poseSet.headings(), constraints);

    }

}
