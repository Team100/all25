package org.team100.frc2025.Swerve.SemiAuto.SemiAuto_i1;

import java.util.ArrayList;
import java.util.List;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.Swerve.SemiAuto.Navigator;
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

public class DriveTo_EF extends Navigator {
    private final SwerveDriveSubsystem m_drive;
    private final TrajectoryFollower m_controller;
    private Pose2d m_goal = new Pose2d();
    private final TrajectoryVisualization m_viz;

    TimingConstraintFactory m_constraints;

    public DriveTo_EF(
            LoggerFactory log,
            SwerveDriveSubsystem robotDrive,
            TrajectoryFollower controller,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics) {
        super(log, robotDrive, controller, viz, kinodynamics);
        m_drive = robotDrive;
        m_controller = controller;
        m_viz = viz;
        m_constraints = new TimingConstraintFactory(kinodynamics);
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Pose2d currPose = m_drive.getPose();
        FieldConstants.FieldSector originSector = FieldConstants.getSector(currPose);
        FieldConstants.FieldSector destinationSector = FieldConstants.FieldSector.EF;
        FieldConstants.ReefDestination destinationPoint = FieldConstants.ReefDestination.CENTER;

        List<Pose2d> waypointsM = new ArrayList<>();
        List<Rotation2d> headings = new ArrayList<>();

        switch (originSector) {
            case AB:
                waypointsM.add(new Pose2d(FieldConstants.getOrbitWaypoint(FieldConstants.FieldSector.CD),
                        Rotation2d.fromDegrees(-10)));

                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                        Rotation2d.fromDegrees(60)));

                headings.add(Rotation2d.fromDegrees(60));

                headings.add(Rotation2d.fromDegrees(120));

                break;
            case CD:

                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                        Rotation2d.fromDegrees(60)));

                headings.add(Rotation2d.fromDegrees(120));
                break;

            case EF:
                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                        Rotation2d.fromDegrees(120)));

                headings.add(Rotation2d.fromDegrees(120));
                break;

            case GH:
                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                        Rotation2d.fromDegrees(210)));

                headings.add(Rotation2d.fromDegrees(120));
                break;
            case IJ:
                waypointsM.add(new Pose2d(FieldConstants.getOrbitWaypoint(FieldConstants.FieldSector.GH),
                        Rotation2d.fromDegrees(270)));

                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                        Rotation2d.fromDegrees(210)));

                headings.add(Rotation2d.fromDegrees(180));

                headings.add(Rotation2d.fromDegrees(120));
                break;
            case KL:

                waypointsM.add(new Pose2d(FieldConstants.getOrbitWaypoint(FieldConstants.FieldSector.IJ),
                        Rotation2d.fromDegrees(0)));

                waypointsM.add(new Pose2d(FieldConstants.getOrbitWaypoint(FieldConstants.FieldSector.GH),
                        Rotation2d.fromDegrees(270)));

                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint),
                        Rotation2d.fromDegrees(210)));

                headings.add(Rotation2d.fromDegrees(-120));

                headings.add(Rotation2d.fromDegrees(180));

                headings.add(Rotation2d.fromDegrees(120));
                break;
            default:
                break;

        }

        // m_goal = waypointsM.get(waypointsM.size() - 1);
        // m_log.m_log_goal.log(() -> m_goal);

        PoseSet poseSet = addRobotPose(currPose, waypointsM, headings);
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(poseSet.poses(), poseSet.headings(),
                m_constraints.fast());
        m_viz.setViz(trajectory);
        m_controller.setTrajectory(trajectory);

    }

}
