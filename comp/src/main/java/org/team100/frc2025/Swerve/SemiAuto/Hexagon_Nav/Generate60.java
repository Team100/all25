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

public class Generate60 extends Navigator {

    private final double kTangentScale = 1;
    private final double kEntranceCurveFactor = 0.25;

    private final FieldSector m_end;
    private final ReefDestination m_reefDestination;

    public Generate60(
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
        ReefAproach approach = path.approach();


        List<Pose2d> waypointsM = new ArrayList<>();
        List<Rotation2d> headings = new ArrayList<>();


        Rotation2d initialSpline = destination.minus(currTranslation).getAngle();

        Rotation2d endSpline = new Rotation2d(0);

    

        Rotation2d anchorPointRotation = FieldConstants.calculateAnchorPointDelta(FieldConstants.getSectorAngle(start),
                approach);
        Translation2d anchorWaypoint = FieldConstants.getOrbitWaypoint(anchorPointRotation, 2.4);

        Translation2d pointToAnchor = anchorWaypoint.minus(FieldConstants.getReefCenter());
       

        Translation2d vectorToAnchorPoint = anchorWaypoint.minus(currTranslation);



        double dotProduct = ((pointToAnchor.getX()/pointToAnchor.getNorm()) * (vectorToAnchorPoint.getX()/vectorToAnchorPoint.getNorm()))
                + ((pointToAnchor.getY()/pointToAnchor.getNorm()) * (vectorToAnchorPoint.getY()/vectorToAnchorPoint.getNorm()));

        // System.out.println(dotProduct);


        double distanceToReef = FieldConstants.getDistanceToReefCenter(currTranslation);


        if (dotProduct <= 1 && dotProduct >= Math.cos(65 * 180/Math.PI)) {

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

        return m_planner.restToRest(poseSet.poses(), poseSet.headings());

    }

}
