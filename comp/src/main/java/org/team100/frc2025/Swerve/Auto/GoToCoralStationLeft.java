package org.team100.frc2025.Swerve.Auto;

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

public class GoToCoralStationLeft extends Navigator {

    private final double kScale;

    public GoToCoralStationLeft(
            LoggerFactory parent,
            SwerveDriveSubsystem drive,
            SwerveController hcontroller,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics,
            double scale) {
        super(parent, drive, hcontroller, viz, kinodynamics);
        kScale = scale;
    }

    @Override
    public Trajectory100 trajectory(Pose2d currentPose) {

        List<Pose2d> waypointsM = new ArrayList<>();
        List<Rotation2d> headings = new ArrayList<>();

        Translation2d currTranslation = currentPose.getTranslation();
        Translation2d goalTranslation = new Translation2d(1.2, 7.15);

        Rotation2d spline = goalTranslation.minus(currTranslation).getAngle();

        Rotation2d newInitialSpline = FieldConstants.calculateDeltaSpline(spline, spline.rotateBy(Rotation2d.fromDegrees(-90)), null, kScale);

        waypointsM.add(new Pose2d(currTranslation, newInitialSpline));
        waypointsM.add(new Pose2d(goalTranslation, spline));

        headings.add(currentPose.getRotation());
        headings.add(Rotation2d.fromDegrees(-50));



        return m_planner.restToRest(waypointsM, headings);


    }

}
