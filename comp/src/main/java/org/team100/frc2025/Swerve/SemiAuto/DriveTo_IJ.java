package org.team100.frc2025.Swerve.SemiAuto;

import java.util.ArrayList;
import java.util.List;

import org.team100.frc2025.FieldConstants;
import org.team100.lib.follower.DriveTrajectoryFollower;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.PoseSet;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveTo_IJ extends Command implements Planner2025 {
    /** Creates a new TrajectoryCommandWithPose100. */
    public static class Log {
        private final Pose2dLogger m_log_goal;
        private final ChassisSpeedsLogger m_log_chassis_speeds;
        private final DoubleLogger m_log_THETA_ERROR;
        private final BooleanLogger m_log_FINSIHED;

        public Log(LoggerFactory parent) {
            LoggerFactory log = parent.child("DriveToIJ");
            m_log_goal = log.pose2dLogger(Level.TRACE, "goal");
            m_log_chassis_speeds = log.chassisSpeedsLogger(Level.TRACE, "chassis speeds");
            m_log_THETA_ERROR = log.doubleLogger(Level.TRACE, "THETA ERROR");
            m_log_FINSIHED = log.booleanLogger(Level.TRACE, "FINSIHED");
        }
    }

    private final Log m_log;
    private final SwerveDriveSubsystem m_robotDrive;
    private final DriveTrajectoryFollower m_controller;
    private Pose2d m_goal = new Pose2d();
    private final TrajectoryVisualization m_viz;

    
    TimingConstraintFactory m_constraints;

    

    public DriveTo_IJ(
            Log log,
            SwerveDriveSubsystem robotDrive,
            DriveTrajectoryFollower controller,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics) {
        m_log = log;
        m_robotDrive = robotDrive;
        m_controller = controller;
        m_viz = viz;
        m_constraints = new TimingConstraintFactory(kinodynamics);
        addRequirements(m_robotDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d currPose = m_robotDrive.getPose();
        FieldConstants.FieldSector originSector = FieldConstants.getSector(currPose);
        FieldConstants.FieldSector destinationSector = FieldConstants.FieldSector.IJ;
        FieldConstants.ReefPoint destinationPoint = FieldConstants.ReefPoint.CENTER;


        List<Pose2d> waypointsM = new ArrayList<>();;
        List<Rotation2d> headings = new ArrayList<>();;
    
        switch(originSector){
            case AB:

                waypointsM.add(new Pose2d(FieldConstants.getOrbitWaypoint(FieldConstants.FieldSector.KL), Rotation2d.fromDegrees(5)));
                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint), Rotation2d.fromDegrees(-50)));

                
                headings.add(Rotation2d.fromDegrees(-60));
                headings.add(Rotation2d.fromDegrees(-120));

                break;
            case CD:
                waypointsM.add(new Pose2d(FieldConstants.getOrbitWaypoint(Rotation2d.fromDegrees(-90)), Rotation2d.fromDegrees(0)));

                waypointsM.add(new Pose2d(FieldConstants.getOrbitWaypoint(Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(90)));

                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint), Rotation2d.fromDegrees(160)));

                headings.add(Rotation2d.fromDegrees(90));
                headings.add(Rotation2d.fromDegrees(-180));
                headings.add(Rotation2d.fromDegrees(-120));
                break;

                
            case EF:
                waypointsM.add(new Pose2d(FieldConstants.getOrbitWaypoint(Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(90)));

                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint), Rotation2d.fromDegrees(160)));

                headings.add(Rotation2d.fromDegrees(-180));
                headings.add(Rotation2d.fromDegrees(-120));
                break;

            case GH:
                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint), Rotation2d.fromDegrees(170)));
                
                headings.add(Rotation2d.fromDegrees(-120));
                break;
            case IJ:
                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint), Rotation2d.fromDegrees(-120)));
                
                headings.add(Rotation2d.fromDegrees(-120));
                break;
            case KL:

                waypointsM.add(new Pose2d(FieldConstants.getOrbitDestination(destinationSector, destinationPoint), Rotation2d.fromDegrees(-60)));
                
                headings.add(Rotation2d.fromDegrees(-120));
                break;
            default:
                break;
            
        }

        m_goal = waypointsM.get(waypointsM.size() - 1);
        m_log.m_log_goal.log(() -> m_goal);

        PoseSet poseSet = addRobotPose(currPose, waypointsM, headings);
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(poseSet.poses(), poseSet.headings(), m_constraints.fast());
        m_viz.setViz(trajectory);
        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(new TrajectoryTimeSampler(trajectory));
        m_controller.setTrajectory(iter);

        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final double now = Timer.getFPGATimestamp();
        Pose2d currentPose = m_robotDrive.getPose();
        ChassisSpeeds currentRobotRelativeSpeed = m_robotDrive.getChassisSpeeds();

        Rotation2d angleToReef = FieldConstants.angleToReefCenter(currentPose);
        Rotation2d currentHeading = currentPose.getRotation();
        Rotation2d thetaError = angleToReef.minus(currentHeading);

        // m_controller.setThetaError(thetaError);
        ChassisSpeeds output = m_controller.update(now, currentPose, currentRobotRelativeSpeed);

        m_robotDrive.setChassisSpeedsNormally(output);

        m_log.m_log_chassis_speeds.log(() -> output);
        double thetaErrorRad = m_goal.getRotation().getRadians()
                - m_robotDrive.getPose().getRotation().getRadians();
        m_log.m_log_THETA_ERROR.log(() -> thetaErrorRad);
        m_log.m_log_FINSIHED.log(() -> false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_log.m_log_FINSIHED.log(() -> true);
        m_robotDrive.stop();
        m_viz.clear();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_controller.isDone();
        // return false;
    }

    public PoseSet addRobotPose(Pose2d currPose, List<Pose2d> waypoints, List<Rotation2d> headings){
        Translation2d currTranslation = currPose.getTranslation();
        Translation2d firstWaypoint = waypoints.get(0).getTranslation();
        Rotation2d initialSpline = firstWaypoint.minus(currTranslation).getAngle();
        Pose2d initialWaypoint = new Pose2d(currTranslation, initialSpline);
        Rotation2d initialHeading = currPose.getRotation();

        List<Pose2d> waypointsWithPose = new ArrayList<>();
        List<Rotation2d> headingsWithPose = new ArrayList<>();;

        waypointsWithPose.addAll(waypoints);
        headingsWithPose.addAll(headings);

        waypointsWithPose.add(0, initialWaypoint);
        headingsWithPose.add(0, initialHeading);

        return new PoseSet(waypointsWithPose, headingsWithPose);
    }
}
