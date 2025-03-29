package org.team100.frc2025.Swerve.SemiAuto;

import java.util.ArrayList;
import java.util.List;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.FieldConstants.ReefAproach;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class Navigator extends Command implements Planner2025 {

    public final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final TrajectoryVisualization m_viz;

    // used by trajectory()
    protected final TrajectoryPlanner m_planner;

    // created in initialize()
    protected ReferenceController m_referenceController;

    public Navigator(
            LoggerFactory parent,
            SwerveDriveSubsystem drive,
            SwerveController controller,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics) {
        m_drive = drive;
        m_controller = controller;
        m_viz = viz;
        m_planner = new TrajectoryPlanner(new TimingConstraintFactory(kinodynamics).medium());
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Trajectory100 trajectory = trajectory(m_drive.getPose());
        m_viz.setViz(trajectory);

        m_referenceController = new ReferenceController(
                m_drive,
                m_controller,
                new TrajectoryReference(trajectory), true);
    }

    /** Subclasses make trajectories. */
    protected abstract Trajectory100 trajectory(Pose2d currentPose);

    @Override
    public final void execute() {
        m_referenceController.execute();
    }

    @Override
    public final void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();

        System.out.println("I FINISHED");
    }

    @Override
    public final boolean isFinished() {
        return m_referenceController.isFinished();
        // return false;
    }

    public static List<HolonomicPose2d> addRobotPose(Pose2d currPose, List<HolonomicPose2d> waypoints) {
        Translation2d currTranslation = currPose.getTranslation();
        Translation2d firstWaypoint = waypoints.get(0).translation();
        Rotation2d initialSpline = firstWaypoint.minus(currTranslation).getAngle();

        Rotation2d initialHeading = currPose.getRotation();
        HolonomicPose2d initialWaypoint = new HolonomicPose2d(currTranslation, initialHeading, initialSpline);

        List<HolonomicPose2d> waypointsWithPose = new ArrayList<>();
        waypointsWithPose.add(initialWaypoint);
        waypointsWithPose.addAll(waypoints);
        return waypointsWithPose;
    }

    public static List<HolonomicPose2d> addRobotPose(
            Pose2d currPose,
            List<HolonomicPose2d> waypoints,
            Rotation2d initialSpline) {
        Translation2d currTranslation = currPose.getTranslation();
        Rotation2d initialHeading = currPose.getRotation();
        HolonomicPose2d initialWaypoint = new HolonomicPose2d(
                currTranslation,
                initialHeading,
                initialSpline);
        List<HolonomicPose2d> waypointsWithPose = new ArrayList<>();
        waypointsWithPose.add(initialWaypoint);
        waypointsWithPose.addAll(waypoints);
        return waypointsWithPose;
    }

    public Rotation2d calculateInitialSpline(Translation2d targetPoint, Translation2d currTranslation,
            Translation2d vectorFromCenterToRobot, ReefAproach approach, double magicNumber) {

        double distanceToReef = FieldConstants.getDistanceToReefCenter(currTranslation);

        Translation2d translationToTarget = targetPoint.minus(currTranslation);

        Rotation2d rotationAngle = new Rotation2d();

        switch (approach) {
            case CCW:
                rotationAngle = Rotation2d.fromDegrees(90);
                break;
            case CW:
                rotationAngle = Rotation2d.fromDegrees(-90);
                break;
        }

        Translation2d tangentVector = vectorFromCenterToRobot.rotateBy(rotationAngle);

        // MAGIC NUMBER is a MAGIC NUMBER
        Translation2d tangentVectorAdjusted = tangentVector.times((1 / distanceToReef) * magicNumber);

        Translation2d finalVector = translationToTarget.plus(tangentVectorAdjusted);

        Rotation2d finalAngle = finalVector.getAngle();

        return finalAngle;
    }

    public static Rotation2d calculateInitialSpline(Translation2d targetPoint, Translation2d currTranslation) {
        Rotation2d initialSpline = targetPoint.minus(currTranslation).getAngle();
        return initialSpline;

    }

}
