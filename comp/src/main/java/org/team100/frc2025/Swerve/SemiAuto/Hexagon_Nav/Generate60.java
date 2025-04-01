package org.team100.frc2025.Swerve.SemiAuto.Hexagon_Nav;

import java.util.ArrayList;
import java.util.List;

import org.team100.frc2025.Swerve.SemiAuto.Navigator;
import org.team100.lib.commands.drivetrain.FieldConstants;
import org.team100.lib.commands.drivetrain.ReefPath;
import org.team100.lib.commands.drivetrain.FieldConstants.FieldSector;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefAproach;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefDestination;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
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

        Rotation2d initialSpline = destination.minus(currTranslation).getAngle();

        Rotation2d endSpline = new Rotation2d(0);

        Rotation2d anchorPointRotation = FieldConstants.calculateAnchorPointDelta(FieldConstants.getSectorAngle(start),
                approach);
        Translation2d anchorWaypoint = FieldConstants.getOrbitWaypoint(anchorPointRotation, 2.4);

        Translation2d pointToAnchor = anchorWaypoint.minus(FieldConstants.getReefCenter());

        Translation2d vectorToAnchorPoint = anchorWaypoint.minus(currTranslation);

        double dotProduct = ((pointToAnchor.getX() / pointToAnchor.getNorm())
                * (vectorToAnchorPoint.getX() / vectorToAnchorPoint.getNorm()))
                + ((pointToAnchor.getY() / pointToAnchor.getNorm())
                        * (vectorToAnchorPoint.getY() / vectorToAnchorPoint.getNorm()));

        // System.out.println(dotProduct);

        double distanceToReef = FieldConstants.getDistanceToReefCenter(currTranslation);

        if (dotProduct <= 1 && dotProduct >= Math.cos(65 * 180 / Math.PI)) {

            endSpline = FieldConstants.calculateDeltaSpline(FieldConstants.getLandingAngle(end, approach),
                    FieldConstants.getSectorAngle(end), approach, -0.25);

            initialSpline = FieldConstants.calculateDeltaSpline(initialSpline, FieldConstants.getSectorAngle(start),
                    approach, (1 / (distanceToReef)));

        } else {
            endSpline = initialSpline;
        }

        double distance = 0.15; // Distance to move
        double newX = currTranslation.getX() + (distance * initialSpline.getCos());
        double newY = currTranslation.getY() + (distance * initialSpline.getSin());

        Translation2d newTranslation = new Translation2d(newX, newY);
        List<HolonomicPose2d> waypoints = new ArrayList<>();
        waypoints.add(new HolonomicPose2d(
                newTranslation,
                FieldConstants.angleToReefCenter(newTranslation),
                initialSpline));
        waypoints.add(new HolonomicPose2d(
                destination,
                FieldConstants.getSectorAngle(end).rotateBy(Rotation2d.k180deg),
                endSpline));

        List<HolonomicPose2d> poseSet = addRobotPose(currentPose, waypoints, initialSpline);

        return m_planner.restToRest(poseSet);

    }

}
