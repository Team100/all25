package org.team100.frc2025;

import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.team100.frc2025.Climber.Climber;
import org.team100.frc2025.Climber.ClimberFactory;
import org.team100.frc2025.Climber.ClimberRotate;
import org.team100.frc2025.Swerve.FullCycle;
import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.commands.drivetrain.DriveToPoseSimple;
import org.team100.lib.commands.drivetrain.DriveToPoseWithProfile;
import org.team100.lib.commands.drivetrain.DriveToTranslationWithFront;
import org.team100.lib.commands.drivetrain.FullCycle2;
import org.team100.lib.commands.drivetrain.ResetPose;
import org.team100.lib.commands.drivetrain.Rotate;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.commands.drivetrain.manual.DriveManually;
import org.team100.lib.commands.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.commands.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.commands.drivetrain.manual.ManualWithFullStateHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithProfiledHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithTargetLock;
import org.team100.lib.commands.drivetrain.manual.SimpleManualModuleStates;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.hid.OperatorControlProxy;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.localization.VisionDataProvider24;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LevelPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.sensors.GyroFactory;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Try to keep this container clean; if there's something you want to keep but
 * don't need right now, cut and paste it into {@link RobotContainerParkingLot}.
 */
// for background on drive current limits:
public class RobotContainer implements Glassy {
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    // https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
    private static final double kDriveCurrentLimit = 50;
    private static final double kDriveStatorLimit = 100;

    private final SwerveModuleCollection m_modules;
    // private final Command m_auton;

    // SUBSYSTEMS
    final SwerveDriveSubsystem m_drive;
    final Climber m_climber;

    public RobotContainer(TimedRobot100 robot) throws IOException {
        final AsyncFactory asyncFactory = new AsyncFactory(robot);
        final Async async = asyncFactory.get();
        final Logging logging = Logging.instance();
        final LevelPoller poller = new LevelPoller(async, logging::setLevel, Level.TRACE);
        Util.printf("Using log level %s\n", poller.getLevel().name());
        Util.println("Do not use TRACE in comp, with NT logging, it will overrun");

        final LoggerFactory fieldLogger = logging.fieldLogger;
        final FieldLogger.Log fieldLog = new FieldLogger.Log(fieldLogger);

        final LoggerFactory logger = logging.rootLogger;

        final TrajectoryVisualization viz = new TrajectoryVisualization(fieldLogger);
        final DriverControl driverControl = new DriverControlProxy(logger, async);
        final OperatorControl operatorControl = new OperatorControlProxy(async);
        final SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();

        final LoggerFactory driveLog = logger.child("Drive");
        final LoggerFactory comLog = logger.child("Commands");

        m_modules = SwerveModuleCollection.get(
                driveLog,
                kDriveCurrentLimit,
                kDriveStatorLimit,
                swerveKinodynamics);
        final Gyro gyro = GyroFactory.get(
                driveLog,
                swerveKinodynamics,
                m_modules);

        // ignores the rotation derived from vision.
        final SwerveDrivePoseEstimator100 poseEstimator = swerveKinodynamics.newPoseEstimator(
                driveLog,
                gyro,
                m_modules.positions(),
                GeometryUtil.kPoseZero,
                Takt.get());

        final AprilTagFieldLayoutWithCorrectOrientation m_layout = new AprilTagFieldLayoutWithCorrectOrientation();
        final VisionDataProvider24 visionDataProvider = new VisionDataProvider24(
                driveLog,
                m_layout,
                poseEstimator);

        final AsymSwerveSetpointGenerator setpointGenerator = new AsymSwerveSetpointGenerator(
                driveLog,
                swerveKinodynamics,
                RobotController::getBatteryVoltage);
        final SwerveLocal swerveLocal = new SwerveLocal(
                driveLog,
                swerveKinodynamics,
                setpointGenerator,
                m_modules);

        SwerveLimiter limiter = new SwerveLimiter(swerveKinodynamics, RobotController::getBatteryVoltage);
        m_drive = new SwerveDriveSubsystem(
                fieldLogger,
                driveLog,
                gyro,
                poseEstimator,
                swerveLocal,
                visionDataProvider,
                limiter);

        m_climber = ClimberFactory.get(logger);

        ///////////////////////////
        //
        // DRIVE CONTROLLERS
        //

        final SwerveController holonomicController = SwerveControllerFactory.byIdentity(comLog);

        final DriveManually driveManually = new DriveManually(driverControl::velocity, m_drive);
        final LoggerFactory manLog = comLog.child(driveManually);

        final Feedback100 thetaFeedback = new PIDFeedback(
                manLog, 3.0, 0, 0, true, 0.05, 1);

        driveManually.register("MODULE_STATE", false,
                new SimpleManualModuleStates(manLog, swerveKinodynamics));

        driveManually.register("ROBOT_RELATIVE_CHASSIS_SPEED", false,
                new ManualChassisSpeeds(manLog, swerveKinodynamics));

        driveManually.register("FIELD_RELATIVE_TWIST", false,
                new ManualFieldRelativeSpeeds(manLog, swerveKinodynamics));

        driveManually.register("SNAPS_PROFILED", true,
                new ManualWithProfiledHeading(
                        manLog,
                        swerveKinodynamics,
                        driverControl::desiredRotation,
                        thetaFeedback));

        driveManually.register("SNAPS_FULL_STATE", true,
                new ManualWithFullStateHeading(
                        manLog,
                        swerveKinodynamics,
                        driverControl::desiredRotation,
                        new double[] {
                                5,
                                0.35
                        }));

        driveManually.register("LOCKED", false,
                new ManualWithTargetLock(
                        fieldLog,
                        manLog,
                        swerveKinodynamics,
                        driverControl::target,
                        thetaFeedback));

        // DEFAULT COMMANDS
        m_drive.setDefaultCommand(driveManually);
        if (m_climber != null) {
            m_climber.setDefaultCommand(new ClimberRotate(m_climber, 0.2,
                    operatorControl::ramp));
        }

        // ObjectPosition24ArrayListener objectPosition24ArrayListener = new
        // ObjectPosition24ArrayListener(poseEstimator);

        // DRIVER BUTTONS
        final HolonomicProfile profile = new HolonomicProfile(
                swerveKinodynamics.getMaxDriveVelocityM_S() * 0.5,
                swerveKinodynamics.getMaxDriveAccelerationM_S2() * 0.25,
                0.01, // 1 cm
                swerveKinodynamics.getMaxAngleSpeedRad_S() * 0.5,
                swerveKinodynamics.getMaxAngleAccelRad_S2() * 0.02,
                0.1); // 5 degrees

        whileTrue(driverControl::driveToObject,
                new DriveToPoseWithProfile(
                        fieldLog,
                        () -> (Optional.of(m_layout.getTagPose(DriverStation.getAlliance().get(), 16).get().toPose2d()
                                .plus(new Transform2d(0, -2.5, new Rotation2d(Math.PI / 2))))),
                        m_drive,
                        holonomicController,
                        profile));

        whileTrue(driverControl::driveOneMeter,
                new DriveToPoseWithProfile(
                        fieldLog,
                        () -> (Optional.of(m_layout.getTagPose(DriverStation.getAlliance().get(), 16).get().toPose2d()
                                .plus(new Transform2d(0, -3.5, new Rotation2d(Math.PI / 2))))),
                        m_drive,
                        holonomicController,
                        profile));
        whileTrue(driverControl::never,
                new DriveToTranslationWithFront(
                        fieldLog,
                        () -> Optional.of(new Translation2d(1, 4)),
                        m_drive,
                        holonomicController,
                        profile));
        whileTrue(driverControl::fullCycle,
                new FullCycle(manLog, m_drive, viz, swerveKinodynamics, holonomicController));

        whileTrue(driverControl::test,
                new FullCycle2(manLog, m_drive, viz, swerveKinodynamics, holonomicController));

        // test driving without profiling
        whileTrue(driverControl::button4,
                new DriveToPoseSimple(SwerveControllerFactory.ridiculous(manLog), m_drive, new SwerveModel()));
        // test rotating in place
        whileTrue(driverControl::button5,
                new Rotate(m_drive, holonomicController, swerveKinodynamics, Math.PI / 2));

        onTrue(driverControl::resetRotation0, new ResetPose(m_drive, new Pose2d()));
        onTrue(driverControl::resetRotation180, new SetRotation(m_drive, GeometryUtil.kRotation180));

        // OPERATOR BUTTONS
        // whileTrue(operatorControl::outtake, new ClimberRotate(m_climber, -0.2 ));

    }

    public void beforeCommandCycle() {
        // ModeSelector.selectMode(operatorControl::pov);
    }

    public void onTeleop() {
        // m_shooter.reset();
    }

    public void onInit() {
        // m_drive.resetPose()
        m_drive.resetPose(new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d(Math.PI)));

    }

    public void onAuto() {
        // m_drive.resetPose(new Pose2d(m_drive.getPose().getTranslation(), new
        // Rotation2d())
    }

    private void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }

    private void onTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).onTrue(command);
    }

    public void scheduleAuton() {
        // if (m_auton == null)
        // return;
        // m_auton.schedule();
    }

    public void periodic() {
        //
    }

    public void cancelAuton() {
        // if (m_auton == null)
        // return;
        // m_auton.cancel();
    }

    // this keeps the tests from conflicting via the use of simulated HAL ports.
    public void close() {
        m_modules.close();
    }

}
