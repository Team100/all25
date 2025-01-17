package org.team100.frc2024;

import java.io.IOException;
import java.util.Map;
import java.util.function.BooleanSupplier;

import org.team100.frc2024.Climber.Climber;
import org.team100.frc2024.Climber.ClimberRotate;
import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.commands.drivetrain.DriveToPoseSimple;
import org.team100.lib.commands.drivetrain.FancyTrajectory;
import org.team100.lib.commands.drivetrain.ResetPose;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.commands.drivetrain.for_testing.OscillateProfile;
import org.team100.lib.commands.drivetrain.manual.DriveManually;
import org.team100.lib.commands.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.commands.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.commands.drivetrain.manual.ManualWithFullStateHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithProfiledHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithTargetLock;
import org.team100.lib.commands.drivetrain.manual.SimpleManualModuleStates;
import org.team100.lib.controller.drivetrain.FullStateDriveController;
import org.team100.lib.controller.drivetrain.HolonomicDriveControllerFactory;
import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.controller.drivetrain.MinTimeDriveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.follower.DrivePIDFFollower;
import org.team100.lib.follower.DriveTrajectoryFollower;
import org.team100.lib.follower.DriveTrajectoryFollowerFactory;
import org.team100.lib.follower.DriveTrajectoryFollowerUtil;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.hid.OperatorControlProxy;
import org.team100.lib.indicator.LEDIndicator;
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
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.sensors.GyroFactory;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Try to keep this container clean; if there's something you want to keep but
 * don't need right now, cut and paste it into {@link RobotContainerParkingLot}.
 */
public class RobotContainer implements Glassy {
    // for background on drive current limits:
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    // https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
    private static final double kDriveCurrentLimit = 50;
    private static final double kDriveStatorLimit = 100;

    private final SwerveModuleCollection m_modules;
    // private final Command m_auton;

    //SUBSYSTEMS
    final SwerveDriveSubsystem m_drive;
    final Climber m_climber;

    public RobotContainer(TimedRobot100 robot) throws IOException {
        final AsyncFactory asyncFactory = new AsyncFactory(robot);
        final Async async = asyncFactory.get();
        final Logging logging = Logging.instance();
        final LevelPoller poller = new LevelPoller(async, logging::setLevel, Level.COMP);
        Util.printf("Using log level %s\n", poller.getLevel().name());
        Util.println("Do not use TRACE in comp, with NT logging, it will overrun");

        final LoggerFactory fieldLogger = logging.fieldLogger;
        final FieldLogger.Log fieldLog = new FieldLogger.Log(fieldLogger);

        final LoggerFactory logger = logging.rootLogger;

        final TrajectoryVisualization viz = new TrajectoryVisualization(fieldLogger);
        final DriverControl driverControl = new DriverControlProxy(logger, async);
        final OperatorControlProxy operatorControl = new OperatorControlProxy(async);
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
                Timer.getFPGATimestamp());

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

        m_drive = new SwerveDriveSubsystem(
                fieldLogger,
                driveLog,
                gyro,
                poseEstimator,
                swerveLocal,
                visionDataProvider);


        m_climber = new Climber();

       

        ///////////////////////////
        //
        // DRIVE CONTROLLERS
        //

        HolonomicFieldRelativeController.Log hlog = new HolonomicFieldRelativeController.Log(comLog);

        final DriveTrajectoryFollowerUtil util = new DriveTrajectoryFollowerUtil(comLog);
        final DriveTrajectoryFollowerFactory driveControllerFactory = new DriveTrajectoryFollowerFactory(util);
        DrivePIDFFollower.Log PIDFlog = new DrivePIDFFollower.Log(comLog);

        
        final PIDController thetaController = new PIDController(3.0, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        final PIDController omegaController = new PIDController(0.2, 0, 0);

        final DriveManually driveManually = new DriveManually(driverControl::velocity, m_drive);
        final LoggerFactory manLog = comLog.child(driveManually);

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
                        thetaController,
                        omegaController));

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
                        thetaController,
                        omegaController,
                        driverControl::trigger));

        
        // DEFAULT COMMANDS
        m_drive.setDefaultCommand(driveManually);
        m_climber.setDefaultCommand(new ClimberRotate(m_climber, 0.2, operatorControl::climb));


        //DRIVER BUTTONS
        onTrue(driverControl::resetRotation0, new ResetPose(m_drive, 0, 0, 0));
        onTrue(driverControl::resetRotation180, new SetRotation(m_drive, GeometryUtil.kRotation180));
        whileTrue(driverControl::driveWithFancyTrajec,
                new FancyTrajectory(
                        comLog,
                        m_drive,
                        driveControllerFactory.fancyPIDF(PIDFlog),
                        swerveKinodynamics));
        
        
        //OPERATOR BUTTONS
        // whileTrue(operatorControl::ramp, new ClimberRotate(m_climber, 0.2 ));
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
        //     return;
        // m_auton.schedule();
    }

    public void periodic() {
        //
    }

    public void cancelAuton() {
        // if (m_auton == null)
        //     return;
        // m_auton.cancel();
    }

    // this keeps the tests from conflicting via the use of simulated HAL ports.
    public void close() {
        m_modules.close();
    }

}
