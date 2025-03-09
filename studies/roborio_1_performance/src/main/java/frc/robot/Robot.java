package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.localization.VisionDataProvider24;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.sensors.GyroFactory;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.util.Memo;
import org.team100.lib.util.Takt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    TrajectoryPlanner m_planner;
    SwerveDriveSubsystem m_drive;
    SwerveController holonomicController;
    ReferenceController m_referenceController;
    SwerveModuleCollection m_modules;
    ScheduledExecutorService m_initializer;

    public Robot() {
        Logging logging = Logging.instance();

        LoggerFactory logger = logging.rootLogger;
        LoggerFactory fieldLogger = logging.fieldLogger;

        LoggerFactory log = logger.child("Commands");
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get(() -> 0.5);
        m_modules = SwerveModuleCollection.get(
                log,
                60,
                90,
                swerveKinodynamics);
        Gyro gyro = GyroFactory.get(
                log,
                swerveKinodynamics,
                m_modules);
        SwerveDrivePoseEstimator100 poseEstimator = swerveKinodynamics.newPoseEstimator(
                log,
                gyro,
                m_modules.positions(),
                Pose2d.kZero,
                Takt.get());
        SwerveLocal swerveLocal = new SwerveLocal(
                log,
                swerveKinodynamics,
                m_modules);
        AprilTagFieldLayoutWithCorrectOrientation m_layout;
        try {
            m_layout = new AprilTagFieldLayoutWithCorrectOrientation();

            VisionDataProvider24 visionDataProvider = new VisionDataProvider24(
                    log,
                    m_layout,
                    poseEstimator);

            SwerveLimiter limiter = new SwerveLimiter(swerveKinodynamics, RobotController::getBatteryVoltage);

            m_drive = new SwerveDriveSubsystem(
                    fieldLogger,
                    log,
                    gyro,
                    poseEstimator,
                    swerveLocal,
                    visionDataProvider,
                    limiter);

            holonomicController = SwerveControllerFactory.byIdentity(log);
            m_planner = new TrajectoryPlanner(new TimingConstraintFactory(swerveKinodynamics).medium());

            m_initializer = Executors.newSingleThreadScheduledExecutor();
            m_initializer.schedule(this::initStuff, 0, TimeUnit.SECONDS);
            m_initializer.schedule(System::gc, 3, TimeUnit.SECONDS);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void initStuff() {
        // do some initialization to save time later.
        List<HolonomicPose2d> waypoints = new ArrayList<>();
        waypoints.add(new HolonomicPose2d(
                new Translation2d(),
                Rotation2d.kZero,
                Rotation2d.kZero));
        waypoints.add(new HolonomicPose2d(
                new Translation2d(1, 0),
                Rotation2d.kZero,
                Rotation2d.kZero));
        double startS = Takt.actual();
        m_planner.restToRest(waypoints);
        double endS = Takt.actual();
        System.out.printf("ET5 %f\n", endS - startS);

        startS = Takt.actual();
        m_drive.driveInFieldCoords(new FieldRelativeVelocity(0, 0, 0));
        endS = Takt.actual();
        System.out.printf("ET6 %f\n", endS - startS);

        startS = Takt.actual();
        System.gc();
        endS = Takt.actual();
        System.out.printf("ET7 %f\n", endS - startS);
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Memo.resetAll();
        Memo.updateAll();
        CommandScheduler.getInstance().run();

    }

    // 147ms
    @Override
    public void teleopInit() {
        // 259ms first time, then 2ms
        double startS = Takt.actual();
        m_drive.driveInFieldCoords(new FieldRelativeVelocity(0, 0, 0));
        double endS = Takt.actual();
        System.out.printf("ET3 %f\n", endS - startS);

        // 136ms first time, then 9
        startS = Takt.actual();
        Trajectory100 trajectory = trajectory(m_drive.getPose());
        endS = Takt.actual();
        System.out.printf("ET4 %f\n", endS - startS);

        // 6 ms then 0.05 ms
        startS = Takt.actual();
        m_referenceController = new ReferenceController(
                m_drive,
                holonomicController,
                new TrajectoryReference(trajectory),
                true);
        endS = Takt.actual();
        System.out.printf("ET2 %f\n", endS - startS);
    }

    // 314ms
    @Override
    public void teleopPeriodic() {
        double startS = Takt.actual();
        m_referenceController.execute();
        double endS = Takt.actual();
        System.out.printf("ET1 %f\n", endS - startS);
    }

    @Override
    public void teleopExit() {
    }

    Trajectory100 trajectory(Pose2d currentPose) {

        // Translation2d currTranslation = currentPose.getTranslation();
        // Translation2d goalTranslation = FieldConstants.getOrbitDestination(m_end,
        // m_reefDestination, 1.3);

        // Rotation2d bearingToGoal = goalTranslation.minus(currTranslation).getAngle();

        // List<HolonomicPose2d> waypoints = new ArrayList<>();
        // waypoints.add(new HolonomicPose2d(
        // currTranslation,
        // currentPose.getRotation(),
        // bearingToGoal));
        // waypoints.add(new HolonomicPose2d(
        // goalTranslation,
        // FieldConstants.getSectorAngle(m_end).rotateBy(Rotation2d.fromDegrees(180)),
        // bearingToGoal));

        // return m_planner.restToRest(waypoints);

        Translation2d currTranslation = currentPose.getTranslation();
        Translation2d goalTranslation = new Translation2d(1, 1);

        Rotation2d bearingToGoal = goalTranslation.minus(currTranslation).getAngle();

        List<HolonomicPose2d> waypoints = new ArrayList<>();
        waypoints.add(new HolonomicPose2d(
                currTranslation,
                currentPose.getRotation(),
                bearingToGoal));
        waypoints.add(new HolonomicPose2d(
                goalTranslation,
                currentPose.getRotation(),
                bearingToGoal));

        // 132 ms then 7 ms
        double startS = Takt.actual();
        Trajectory100 restToRest = m_planner.restToRest(waypoints);
        double endS = Takt.actual();
        System.out.printf("ET0 %f\n", endS - startS);
        return restToRest;

    }
}
