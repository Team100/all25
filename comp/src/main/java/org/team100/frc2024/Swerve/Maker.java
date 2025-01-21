package org.team100.frc2024.Swerve;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.team100.frc2024.FieldConstants;
import org.team100.lib.commands.drivetrain.TrajectoryCommand100;
import org.team100.lib.follower.DrivePIDFFollower;
import org.team100.lib.follower.DriveTrajectoryFollower;
import org.team100.lib.follower.DriveTrajectoryFollowerFactory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class Maker {

    TimingConstraintFactory constraints;
    DrivePIDFFollower.Log m_PIDFLog;
    LoggerFactory m_logger;
    DriveToS4.Log m_commandLog;
    SwerveDriveSubsystem m_swerve;
    DriveTrajectoryFollowerFactory m_factory;
    TrajectoryVisualization m_viz;
    SwerveKinodynamics m_kinodynamics;

    Trajectory100 m_clockWiseTraj;

    public Maker(LoggerFactory parent, SwerveDriveSubsystem swerve,  DriveTrajectoryFollowerFactory factory, SwerveKinodynamics kinodynamics, TrajectoryVisualization viz){
        m_logger = parent.child("Maker");
        m_PIDFLog = new DrivePIDFFollower.Log(m_logger);
        m_commandLog = new DriveToS4.Log(m_logger);
        m_factory = factory;
        m_swerve = swerve;
        constraints = new TimingConstraintFactory(kinodynamics);
        m_viz = viz;
        m_kinodynamics = kinodynamics;

        makeClockWiseTraj();
    }


    public Pose2d makeTrajectoryCommand(Supplier<Pose2d> robotPose) {

        Trajectory100 trajectory = m_clockWiseTraj;

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(new TrajectoryTimeSampler(trajectory));

        // Optional<TrajectorySamplePoint> futurePoint = iter.advance(0.4);

        double[] center  = { FieldConstants.getReefCenter().getX(), FieldConstants.getReefCenter().getY()};
        double radius = FieldConstants.getOrbitRadius();
        double[] point = {robotPose.get().getX(), robotPose.get().getY()};

        List<double[]> array = getTangentPointsAtCircle(FieldConstants.getReefCenter().getX(), FieldConstants.getReefCenter().getY(), radius, robotPose.get().getX(), robotPose.get().getY());
        
        double[] arr = array.get(0);
        return new Pose2d( new Translation2d(arr[0], arr[1]), new Rotation2d());


        

        

    }

    public void makeClockWiseTraj() {
       
        Translation2d reefCenter = FieldConstants.getReefCenter();
        double circleRadius = FieldConstants.getOrbitRadius();

        Pose2d radian_0 = new Pose2d(reefCenter.getX() + circleRadius, reefCenter.getY(), Rotation2d.fromDegrees(270));
        Pose2d radian_90 = new Pose2d(reefCenter.getX(), reefCenter.getY() - circleRadius, Rotation2d.fromDegrees(180)); 

        Pose2d radian_180 = new Pose2d(reefCenter.getX() - circleRadius, reefCenter.getY(), Rotation2d.fromDegrees(90)); 
        Pose2d radian_270 = new Pose2d(reefCenter.getX(),  reefCenter.getY() + circleRadius, Rotation2d.fromDegrees(0)); 
        Pose2d radian_360 = radian_0;

        List<Pose2d> waypointsM = List.of(
            radian_0,
            radian_90,
            radian_180,
            radian_270,
            radian_360
        );

        List<Rotation2d> headings = List.of(
                new Rotation2d(),
                new Rotation2d(),
                new Rotation2d(),
                new Rotation2d(),
                new Rotation2d()
        );

        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, constraints.fast());

        m_clockWiseTraj = trajectory;

        // return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory, m_factory.goodPIDF(m_PIDFLog),
        //         m_viz);

    }

    public DriveToS4 test() {

        Pose2d waypoint1 = new Pose2d(5.37, 6, Rotation2d.fromDegrees(315));
        Pose2d waypoint2 = new Pose2d(6.305274, 4.074270, Rotation2d.fromDegrees(270));

        List<Pose2d> waypointsM = new ArrayList<>(List.of(
            waypoint1,
            waypoint2
        ));

        List<Rotation2d> headings = new ArrayList<>(List.of(
            new Rotation2d(),
            new Rotation2d()
        ));
        
        return new DriveToS4(
                m_commandLog, 
                m_swerve, 
                waypointsM,
                headings, 
                m_factory.goodPIDF(m_PIDFLog),
                m_viz,
                m_kinodynamics);

    }

    

    // public TrajectoryCommand100 runClockWiseTraj() {
       
    //     Translation2d reefCenter = FieldConstants.getReefCenter();
    //     double circleRadius = FieldConstants.getOrbitRadius();

    //     Pose2d radian_0 = new Pose2d(reefCenter.getX() + circleRadius, reefCenter.getY(), Rotation2d.fromDegrees(270));
    //     Pose2d radian_90 = new Pose2d(reefCenter.getX(), reefCenter.getY() - circleRadius, Rotation2d.fromDegrees(180)); 

    //     Pose2d radian_180 = new Pose2d(reefCenter.getX() - circleRadius, reefCenter.getY(), Rotation2d.fromDegrees(90)); 
    //     Pose2d radian_270 = new Pose2d(reefCenter.getX(),  reefCenter.getY() + circleRadius, Rotation2d.fromDegrees(0)); 
    //     Pose2d radian_360 = radian_0;

    //     List<Pose2d> waypointsM = List.of(
    //         radian_0,
    //         radian_90,
    //         radian_180,
    //         radian_270,
    //         radian_360
    //     );

    //     List<Rotation2d> headings = List.of(
    //             new Rotation2d(),
    //             new Rotation2d(),
    //             new Rotation2d(),
    //             new Rotation2d(),
    //             new Rotation2d()
    //     );

    //     Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, constraints.fast());

    //     m_clockWiseTraj = trajectory;

    //     return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory, m_factory.goodPIDF(m_PIDFLog),
    //             m_viz);

    // }

    public static List<double[]> getTangentPointsAtCircle(double centerX, double centerY, double r, double xx, double yy) {
        List<double[]> tangentPoints = new ArrayList<>();

        if (r == 0) {  // If the radius is 0, no tangents
            return tangentPoints;
        }

        // Shift and scale the external point
        double nx = (xx - centerX) / r;
        double ny = (yy - centerY) / r;
        double xy = nx * nx + ny * ny;

        if (Math.abs(xy - 1.0) < 1e-9) {  // Point lies on the circumference, one tangent
            tangentPoints.add(new double[] { xx, yy });
            return tangentPoints;
        }

        if (xy < 1.0) {  // Point lies inside the circle, no tangents
            return tangentPoints;
        }

        // Common case, two tangents
        double d = ny * Math.sqrt(xy - 1);
        double tx0 = (nx - d) / xy;
        double tx1 = (nx + d) / xy;

        if (ny != 0) {  // Common case where ny != 0
            double yt0 = centerY + r * (1 - tx0 * nx) / ny;
            double yt1 = centerY + r * (1 - tx1 * nx) / ny;
            tangentPoints.add(new double[] { centerX + r * tx0, yt0 });
            tangentPoints.add(new double[] { centerX + r * tx1, yt1 });
        } else {  // Point at the center horizontally, Y=0
            d = r * Math.sqrt(1 - tx0 * tx0);
            tangentPoints.add(new double[] { centerX + r * tx0, centerY + d });
            tangentPoints.add(new double[] { centerX + r * tx1, centerY - d });
        }

        return tangentPoints;
    }

    
}
