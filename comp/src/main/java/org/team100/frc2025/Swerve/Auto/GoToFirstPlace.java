package org.team100.frc2025.Swerve.Auto;

import java.util.ArrayList;
import java.util.List;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.Swerve.SemiAuto.Navigator;
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

public class GoToFirstPlace extends Navigator {

    private final double kTangentScale = 1;
    private final double kEntranceCurveFactor = 0.25;

    private final FieldSector m_end;
    private final ReefDestination m_reefDestination;

    public GoToFirstPlace(
            LoggerFactory parent,
            SwerveDriveSubsystem drive,
            SwerveController hcontroller,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics,
            FieldSector endSector,
            ReefDestination reefDest) {
        super(parent, drive, hcontroller, viz, kinodynamics);
        m_end = endSector;
        m_reefDestination = reefDest;
    }

    @Override
    public Trajectory100 trajectory(Pose2d currentPose) {

        Translation2d currTranslation = currentPose.getTranslation();
        Translation2d goalTranslation = FieldConstants.getOrbitDestination(m_end, m_reefDestination, 1.3);
        Translation2d midPointTranslation = new Translation2d(5.62, 2.22);

        Rotation2d bearingToMid = midPointTranslation.minus(currTranslation).getAngle();
        Rotation2d bearingToGoal = goalTranslation.minus(midPointTranslation).getAngle();

        Rotation2d newInitialSpline = FieldConstants.calculateDeltaSpline(bearingToMid,
                bearingToMid.rotateBy(Rotation2d.fromDegrees(-90)), null, -0.5);

        Rotation2d newMidSpline = FieldConstants.calculateDeltaSpline(bearingToGoal,
                bearingToGoal.rotateBy(Rotation2d.fromDegrees(-90)), null, 0.1);

        List<HolonomicPose2d> waypoints = new ArrayList<>();
        waypoints.add(new HolonomicPose2d(
                currTranslation,
                currentPose.getRotation(),
                newInitialSpline));

        waypoints.add(new HolonomicPose2d(
                midPointTranslation,
                FieldConstants.getSectorAngle(m_end).rotateBy(Rotation2d.fromDegrees(180)),
                newMidSpline));

        waypoints.add(new HolonomicPose2d(
                goalTranslation,
                FieldConstants.getSectorAngle(m_end).rotateBy(Rotation2d.fromDegrees(180)),
                newMidSpline));

        return m_planner.restToRest(waypoints);

    }

}
