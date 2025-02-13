package org.team100.frc2025;

import java.io.IOException;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.commands.drivetrain.DriveToWaypoint100;
import org.team100.lib.commands.drivetrain.PermissiveTrajectoryListCommand;
import org.team100.lib.commands.drivetrain.TrajectoryListCommand;
import org.team100.lib.commands.drivetrain.for_testing.DrawSquare;
import org.team100.lib.commands.drivetrain.for_testing.DriveInACircle;
import org.team100.lib.commands.drivetrain.for_testing.DriveInALittleSquare;
import org.team100.lib.commands.drivetrain.for_testing.Oscillate;
import org.team100.lib.commands.drivetrain.for_testing.Spin;
import org.team100.lib.controller.drivetrain.HolonomicDriveControllerFactory;
import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.follower.TrajectoryFollower;
import org.team100.lib.follower.TrajectoryFollowerFactory;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
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
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.sensors.GyroFactory;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.TrajectoryMaker;
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
        final SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();

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
                GeometryUtil.kPoseZero,
                Takt.get());
        final AprilTagFieldLayoutWithCorrectOrientation m_layout = new AprilTagFieldLayoutWithCorrectOrientation();
        final VisionDataProvider24 visionDataProvider = new VisionDataProvider24(
                driveLogger,
                m_layout,
                poseEstimator);
        final AsymSwerveSetpointGenerator setpointGenerator = new AsymSwerveSetpointGenerator(
                driveLogger,
                swerveKinodynamics,
                RobotController::getBatteryVoltage);
        SwerveLocal swerveLocal = new SwerveLocal(driveLogger, swerveKinodynamics, setpointGenerator, m_modules);
        m_drive = new SwerveDriveSubsystem(
                fieldLogger,
                driveLogger,
                m_gyro,
                poseEstimator,
                swerveLocal,
                visionDataProvider);
        HolonomicFieldRelativeController controller = HolonomicDriveControllerFactory.get(driveLogger);

        ///////////////////////

        // little square
        // this should be a field.
        final DriveInALittleSquare m_driveInALittleSquare;

        m_driveInALittleSquare = new DriveInALittleSquare(m_drive);
        whileTrue(driverControl::never, m_driveInALittleSquare);

        ///////////////////////

        whileTrue(driverControl::never, new DriveInACircle(driveLogger, m_drive, controller, -1, viz));
        whileTrue(driverControl::never, new Spin(m_drive, controller));
        whileTrue(driverControl::never, new Oscillate(driveLogger, m_drive));

        ////////////////////////

        // field center, roughly, facing to the left.
        Pose2d goal = new Pose2d(1.877866, 7.749999, GeometryUtil.kRotation90);
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();

        TrajectoryFollower drivePID = TrajectoryFollowerFactory.autoFieldRelativePIDF(driveLogger);
        whileTrue(driverControl::never,
                new DriveToWaypoint100(
                        driveLogger,
                        goal,
                        m_drive,
                        drivePID,
                        swerveKinodynamics,
                        1,
                        viz));

        TrajectoryFollower driveFF = TrajectoryFollowerFactory.fieldRelativeFfOnly(driveLogger);

        whileTrue(driverControl::never,
                new DriveToWaypoint100(
                        driveLogger,
                        goal,
                        m_drive,
                        driveFF,
                        swerveKinodynamics,
                        1,
                        viz));

        //////////////////////

        // calibration

        TrajectoryMaker maker = new TrajectoryMaker(constraints);

        // make a one-meter line
        whileTrue(driverControl::never,
                new TrajectoryListCommand(driveLogger, m_drive, controller,
                        x -> List.of(maker.line(x)), viz));

        // make a one-meter square
        whileTrue(driverControl::never,
                new TrajectoryListCommand(driveLogger, m_drive, controller,
                        maker::square, viz));

        whileTrue(driverControl::test, new TrajectoryListCommand(driveLogger, m_drive, controller,
                null, viz));

        // one-meter square with reset at the corners
        whileTrue(driverControl::never,
                new PermissiveTrajectoryListCommand(driveLogger, m_drive, controller,
                        maker.permissiveSquare(), viz));

        // one-meter square with position and velocity feedback control
        HolonomicFieldRelativeController fscontroller = HolonomicDriveControllerFactory.get(driveLogger);
        whileTrue(driverControl::never,
                new TrajectoryListCommand(driveLogger, m_drive, fscontroller,
                        maker::square, viz));

        // this should be a field.
        final DrawSquare m_drawCircle = new DrawSquare(driveLogger, m_drive, controller, viz);
        whileTrue(driverControl::never, m_drawCircle);
    }

    private void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }

}
