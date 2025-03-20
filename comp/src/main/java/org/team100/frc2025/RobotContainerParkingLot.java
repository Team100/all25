package org.team100.frc2025;

import java.io.IOException;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.commands.drivetrain.DriveWithTrajectoryList;
import org.team100.lib.commands.drivetrain.PermissiveTrajectoryListCommand;
import org.team100.lib.commands.drivetrain.for_testing.DrawSquare;
import org.team100.lib.commands.drivetrain.for_testing.DriveInACircle;
import org.team100.lib.commands.drivetrain.for_testing.DriveInALittleSquare;
import org.team100.lib.commands.drivetrain.for_testing.Oscillate;
import org.team100.lib.commands.drivetrain.for_testing.Spin;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.hid.OperatorControlProxy;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.localization.VisionDataProvider24;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.sensors.GyroFactory;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.util.Takt;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This is stuff cut out of RobotContainer, so that the compiler will still see
 * it, but so that it's not in the prod robot execution path at all.
 */
public class RobotContainerParkingLot implements Glassy {
    // for background on drive current limits:
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    // https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
    private static final double kDriveCurrentLimit = 60;
    private static final double kDriveStatorLimit = 120;

    private final SwerveModuleCollection m_modules;
    final Gyro m_gyro;
    final SwerveDriveSubsystem m_drive;
    final DriverControl driverControl;
    final OperatorControl operatorControl;

    /**
     * Stuff to come back to someday.
     * 
     * The reason to put it here rather than commenting it out is so that it doesn't
     * rot.
     * 
     * @throws IOException
     */
    RobotContainerParkingLot(TimedRobot100 robot) throws IOException {
        Logging logging = Logging.instance();
        final LoggerFactory fieldLogger = logging.fieldLogger;
        final LoggerFactory driveLogger = logging.rootLogger;
        final TrajectoryVisualization viz = new TrajectoryVisualization(fieldLogger);

        final AsyncFactory asyncFactory = new AsyncFactory(robot);
        final Async async = asyncFactory.get();
        driverControl = new DriverControlProxy(driveLogger, async);
        operatorControl = new OperatorControlProxy(async);
        final SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get(() -> 0.1);

        m_modules = SwerveModuleCollection.get(
                driveLogger,
                kDriveCurrentLimit,
                kDriveStatorLimit,
                swerveKinodynamics);
        m_gyro = GyroFactory.get(
                driveLogger,
                swerveKinodynamics,
                m_modules);
        final SwerveDrivePoseEstimator100 poseEstimator = swerveKinodynamics.newPoseEstimator(
                driveLogger,
                m_gyro,
                m_modules.positions(),
                Pose2d.kZero,
                Takt.get());
        final AprilTagFieldLayoutWithCorrectOrientation m_layout = new AprilTagFieldLayoutWithCorrectOrientation();
        final VisionDataProvider24 visionDataProvider = new VisionDataProvider24(
                driveLogger,
                m_layout,
                poseEstimator);
        SwerveLocal swerveLocal = new SwerveLocal(driveLogger, swerveKinodynamics, m_modules);
        SwerveLimiter limiter = new SwerveLimiter(driveLogger, swerveKinodynamics, RobotController::getBatteryVoltage);
        m_drive = new SwerveDriveSubsystem(
                fieldLogger,
                driveLogger,
                m_gyro,
                poseEstimator,
                swerveLocal,
                visionDataProvider::update,
                limiter);
        SwerveController controller = SwerveControllerFactory.byIdentity(driveLogger);

        ///////////////////////

        // little square
        // this should be a field.
        final DriveInALittleSquare m_driveInALittleSquare;

        m_driveInALittleSquare = new DriveInALittleSquare(m_drive);
        whileTrue(() -> false, m_driveInALittleSquare);

        ///////////////////////

        whileTrue(() -> false, new DriveInACircle(driveLogger, m_drive, controller, -1, viz));
        whileTrue(() -> false, new Spin(m_drive, controller));
        whileTrue(() -> false, new Oscillate(driveLogger, m_drive));

        ////////////////////////

        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();

        //////////////////////

        // calibration

        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

        // make a one-meter line
        whileTrue(() -> false,
                new DriveWithTrajectoryList(m_drive, controller,
                        x -> List.of(planner.line(x)), viz));

        // make a one-meter square
        whileTrue(() -> false,
                new DriveWithTrajectoryList(m_drive, controller,
                        planner::square, viz));

        // one-meter square with reset at the corners
        whileTrue(() -> false,
                new PermissiveTrajectoryListCommand(m_drive, controller,
                        planner.permissiveSquare(), viz));

        // whileTrue(driverControl::driveToObject,

        // new DriveToPoseWithProfile(
        // fieldLog,
        // () -> (Optional.of(m_layout.getTagPose(DriverStation.getAlliance().get(),
        // 16).get().toPose2d()
        // .plus(new Transform2d(0, -0.75, new Rotation2d(Math.PI / 2))))),
        // m_drive,
        // holonomicController,
        // profile));
        // new DriveToPoseWithTrajectory(
        // () -> m_layout.getTagPose(DriverStation.getAlliance().get(),
        // 16).get().toPose2d()
        // .plus(new Transform2d(0, -1, new Rotation2d(Math.PI / 2))),
        // m_drive, (start, end) -> planner.movingToRest(start, end),
        // holonomicController, viz));

        // whileTrue(driverControl::driveOneMeter,
        // // new DriveToPoseWithProfile(
        // // fieldLog,
        // // () -> (Optional.of(m_layout.getTagPose(DriverStation.getAlliance().get(),
        // // 16).get().toPose2d()
        // // .plus(new Transform2d(0, -3.5, new Rotation2d(Math.PI / 2))))),
        // // m_drive,
        // // holonomicController,
        // // profile));
        // new DriveToPoseWithTrajectory(
        // () -> m_layout.getTagPose(DriverStation.getAlliance().get(),
        // 16).get().toPose2d()
        // .plus(new Transform2d(0, -3.5, new Rotation2d(Math.PI / 2))),
        // m_drive, (start, end) -> planner.movingToRest(start, end),
        // holonomicController, viz));
        // whileTrue(driverControl::driveOneMeter, new GoToDestinationDirectly(manLog,
        // m_drive, holonomicController, viz, swerveKinodynamics, FieldSector.AB,
        // ReefDestination.CENTER)); //A
        // whileTrue(driverControl::driveOneMeter, new Embark(m_drive,
        // holonomicController, profile, FieldSector.AB, ReefDestination.CENTER)); //A

        // new DriveToPoseWithTrajectory(
        // () -> m_layout.getTagPose(DriverStation.getAlliance().get(),
        // 16).get().toPose2d()
        // .plus(new Transform2d(0, -1, new Rotation2d(Math.PI / 2))),
        // m_drive, (start, end) -> planner.movingToRest(start, end),
        // holonomicController, viz));

        // whileTrue(driverControl::driveOneMeter,
        // // new Embark(m_drive, holonomicController, profile));
        // new DriveToPoseWithProfile(
        // fieldLog,
        // () -> (Optional.of(new Pose2d(5,6.5, new Rotation2d()))),
        // m_drive,
        // holonomicController,
        // profile));

        // new DriveToPoseWithTrajectory(
        // () -> m_layout.getTagPose(DriverStation.getAlliance().get(),
        // 16).get().toPose2d()
        // .plus(new Transform2d(0, -3.5, new Rotation2d(Math.PI / 2))),
        // m_drive, (start, end) -> planner.movingToRest(start, end),
        // holonomicController, viz));

        // whileTrue(driverControl::never,
        // new DriveToTranslationWithFront(
        // fieldLog,
        // () -> Optional.of(new Translation2d(1, 4)),
        // m_drive,
        // holonomicController,
        // profile));
        // whileTrue(driverControl::fullCycle,
        // new FullCycle(fieldLog, manLog, m_drive, viz, m_swerveKinodynamics,
        // holonomicController, profile));

        // m_auton = new FullCycle(fieldLog, manLog, m_drive, viz, m_swerveKinodynamics,
        // holonomicController, profile);

        // whileTrue(driverControl::test,
        // new FullCycle2(manLog, m_drive, viz, m_swerveKinodynamics,
        // holonomicController));

        // // test driving without profiling
        // whileTrue(driverControl::button4,
        // new DriveToPoseSimple(SwerveControllerFactory.ridiculous(manLog), m_drive,
        // new SwerveModel()));
        // // test rotating in place
        // whileTrue(driverControl::button5,
        // new Rotate(m_drive, holonomicController, m_swerveKinodynamics, Math.PI / 2));
        // // this is joel working on moving-entry trajectories.
        // whileTrue(driverControl::testTrajectory,
        // new DriveToPoseWithTrajectory(
        // () -> new Pose2d(3, 3, Rotation2d.kZero),
        // m_drive,
        // (model, pose) -> planner.movingToRest(model, pose),
        // holonomicController,
        // viz));

        // this should be a field.
        final DrawSquare m_drawCircle = new DrawSquare(m_drive, controller, viz);
        whileTrue(() -> false, m_drawCircle);
    }

    private void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }

}
