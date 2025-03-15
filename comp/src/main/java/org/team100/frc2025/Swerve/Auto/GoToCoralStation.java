package org.team100.frc2025.Swerve.Auto;

import java.util.ArrayList;
import java.util.List;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.FieldConstants.CoralStation;
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

public class GoToCoralStation extends Navigator {

    private double kScale;
    private final CoralStation m_station;

    public GoToCoralStation(
            LoggerFactory parent,
            SwerveDriveSubsystem drive,
            SwerveController hcontroller,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics,
            CoralStation station,
            double scale) {
        super(parent, drive, hcontroller, viz, kinodynamics);
        m_station = station;
        kScale = scale;
    }

    @Override
    public Trajectory100 trajectory(Pose2d currentPose) {

        Translation2d currTranslation = currentPose.getTranslation();
        Translation2d goalTranslation;
        Rotation2d goalRotation;

        double scaleAdjust = kScale;

        if (m_station == CoralStation.Left) {
            goalTranslation = new Translation2d(1.2, 7.00);
            goalRotation = Rotation2d.fromDegrees(-54);
            scaleAdjust *= 1;

        } else {
            goalTranslation = new Translation2d(1.2, 1.05);
            goalRotation = Rotation2d.fromDegrees(54);
            scaleAdjust *= -1;
        }

        Rotation2d courseToGoal = goalTranslation.minus(currTranslation).getAngle();

        Rotation2d newInitialSpline = FieldConstants.calculateDeltaSpline(courseToGoal,
                courseToGoal.rotateBy(Rotation2d.fromDegrees(-90)), null, scaleAdjust);

        List<HolonomicPose2d> waypoints = new ArrayList<>();
        waypoints.add(new HolonomicPose2d(currTranslation, currentPose.getRotation(), newInitialSpline));
        waypoints.add(new HolonomicPose2d(goalTranslation, goalRotation, courseToGoal));

        return m_planner.restToRest(waypoints);

    }

}
