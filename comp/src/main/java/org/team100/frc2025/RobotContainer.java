package org.team100.frc2025;
// import org.team100.frc2025.CommandGroups.ScoreAlgae2;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import org.team100.frc2025.FieldConstants.BargeDestination;
import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.Climber.Climber;
import org.team100.frc2025.Climber.ClimberRotate;
import org.team100.frc2025.CommandGroups.ScoreCoral;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.ElevatorDefaultCommand;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.frc2025.Wrist.WristDefaultCommand;
import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.commands.Buttons2025Demo;
import org.team100.lib.commands.drivetrain.DriveToPoseWithProfile;
import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.commands.Buttons2025Demo;
import org.team100.lib.commands.drivetrain.ResetPose;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.commands.drivetrain.manual.DriveManually;
import org.team100.lib.commands.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.commands.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.commands.drivetrain.manual.ManualWithFullStateHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithProfiledHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithTargetLock;
import org.team100.lib.commands.drivetrain.manual.SimpleManualModuleStates;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.hid.OperatorControlProxy;
import org.team100.lib.hid.ThirdControl;
import org.team100.lib.hid.ThirdControlProxy;
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
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.sensors.GyroFactory;
import org.team100.lib.state.Model100;
import org.team100.lib.timing.ConstantConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Try to keep this container clean; if there's something you want to keep but
 * don't need right now, cut and paste it into {@link RobotContainerParkingLot}.
 */
// for background on drive current limits:
public class RobotContainer implements Glassy {
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    // https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
    // https://docs.google.com/document/d/10uXdmu62AFxyolmwtDY8_9UNnci7eVcev4Y64ZS0Aqk
    // https://github.com/frc1678/C2024-Public/blob/17e78272e65a6ce4f87c00a3514c79f787439ca1/src/main/java/com/team1678/frc2024/Constants.java#L195
    // 2/26/25: Joel updated the supply limit to 90A, see 1678 code above. This is
    // essentially unlimited, so you'll need to run some other kind of limiter (e.g.
    // acceleration) to keep from browning out.
    private static final double kDriveCurrentLimit = 90;
    private static final double kDriveStatorLimit = 110;

    private final SwerveModuleCollection m_modules;
    private final Command m_auton;

    // SUBSYSTEMS
    final SwerveDriveSubsystem m_drive;

    final Elevator m_elevator;
    final Wrist2 m_wrist;
    final Climber m_climber;
    final Funnel m_funnel;
    final LEDIndicator m_leds;

    final CoralTunnel m_tunnel;
    final AlgaeGrip m_grip;
    final SwerveKinodynamics m_swerveKinodynamics;

    private final ScheduledExecutorService m_initializer;

    // final AlgaeIntake m_intake;

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
        final LoggerFactory driveLog = logger.child("Drive");
        final LoggerFactory comLog = logger.child("Commands");
        final LoggerFactory elevatorLog = logger.child("Elevator");
        final LoggerFactory coralSequence = logger.child("Coral Sequence");

        final TrajectoryVisualization viz = new TrajectoryVisualization(fieldLogger);
        final DriverControl driverControl = new DriverControlProxy(logger, async);
        final OperatorControl operatorControl = new OperatorControlProxy(async);
        final ThirdControl buttons = new ThirdControlProxy(async);

        Buttons2025Demo demo = new Buttons2025Demo(buttons);
        demo.setup();

        if (Identity.instance.equals(Identity.COMP_BOT)) {
            // m_leds = new LEDIndicator(0);
            // m_leds.setFront(LEDIndicator.State.ORANGE);
            // m_leds.setBack(LEDIndicator.State.RED);
            // m_leds.setFlashing(true);
            m_leds = null;
            m_elevator = new Elevator(elevatorLog, 2, 19);
            m_wrist = new Wrist2(elevatorLog, 9);
            m_tunnel = new CoralTunnel(elevatorLog, 3, 25);
            m_funnel = new Funnel(logger, 23, 14);
            m_grip = new AlgaeGrip(logger);
            m_climber = new Climber(logger, 18);
            // TODO: calibrate the elevator and use it here.
            // swerveKinodynamics = SwerveKinodynamicsFactory
            // .get(() -> VCG.vcg(m_elevator.getPosition()));
            m_swerveKinodynamics = SwerveKinodynamicsFactory.get(() -> 0.5);

        } else {
            m_swerveKinodynamics = SwerveKinodynamicsFactory
                    .get(() -> 1);
            m_grip = new AlgaeGrip(logger);
            m_tunnel = new CoralTunnel(elevatorLog, 3, 25);
            m_elevator = new Elevator(elevatorLog, 2, 19);
            m_wrist = new Wrist2(elevatorLog, 9);
            m_funnel = new Funnel(logger, 23, 14);
            m_climber = new Climber(logger, 18);
            m_leds = null;
        }

        final TrajectoryPlanner planner = new TrajectoryPlanner(
                List.of(new ConstantConstraint(m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                        m_swerveKinodynamics.getMaxDriveAccelerationM_S2() * 0.5)));
        m_modules = SwerveModuleCollection.get(
                driveLog,
                kDriveCurrentLimit,
                kDriveStatorLimit,
                m_swerveKinodynamics);
                
        final Gyro gyro = GyroFactory.get(
                driveLog,
                m_swerveKinodynamics,
                m_modules);

        // ignores the rotation derived from vision.
        final SwerveDrivePoseEstimator100 poseEstimator = m_swerveKinodynamics.newPoseEstimator(
                driveLog,
                gyro,
                m_modules.positions(),
                Pose2d.kZero,
                Takt.get());

        final AprilTagFieldLayoutWithCorrectOrientation m_layout = new AprilTagFieldLayoutWithCorrectOrientation();
        final VisionDataProvider24 visionDataProvider = new VisionDataProvider24(
                driveLog,
                m_layout,
                poseEstimator);

        final SwerveLocal swerveLocal = new SwerveLocal(
                driveLog,
                m_swerveKinodynamics,
                m_modules);

        SwerveLimiter limiter = new SwerveLimiter(m_swerveKinodynamics, RobotController::getBatteryVoltage);

        m_drive = new SwerveDriveSubsystem(
                fieldLogger,
                driveLog,
                gyro,
                poseEstimator,
                swerveLocal,
                visionDataProvider,
                limiter);

        m_auton = new Command() {
        };

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
                new SimpleManualModuleStates(manLog, m_swerveKinodynamics));

        driveManually.register("ROBOT_RELATIVE_CHASSIS_SPEED", false,
                new ManualChassisSpeeds(manLog, m_swerveKinodynamics));

        driveManually.register("FIELD_RELATIVE_TWIST", false,
                new ManualFieldRelativeSpeeds(manLog, m_swerveKinodynamics));

        driveManually.register("SNAPS_PROFILED", true,
                new ManualWithProfiledHeading(
                        manLog,
                        m_swerveKinodynamics,
                        driverControl::desiredRotation,
                        thetaFeedback));

        driveManually.register("SNAPS_FULL_STATE", true,
                new ManualWithFullStateHeading(
                        manLog,
                        m_swerveKinodynamics,
                        driverControl::desiredRotation,
                        new double[] {
                                5,
                                0.35
                        }));

        driveManually.register("LOCKED", false,
                new ManualWithTargetLock(
                        fieldLog,
                        manLog,
                        m_swerveKinodynamics,
                        driverControl::target,
                        thetaFeedback));

        // DEFAULT COMMANDS
        m_drive.setDefaultCommand(driveManually);
      
        m_wrist.setDefaultCommand(new WristDefaultCommand(m_wrist, m_elevator));
        m_elevator.setDefaultCommand(new ElevatorDefaultCommand(m_elevator, m_wrist));
        m_climber.setDefaultCommand(new ClimberRotate(m_climber, 0.2, operatorControl::ramp));

        // DRIVER BUTTONS
        final HolonomicProfile profile = new HolonomicProfile(
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxDriveAccelerationM_S2() * 0.5,
                0.05,
                m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleAccelRad_S2() * 0.2,
                0.1);

        onTrue(driverControl::resetRotation0, new ResetPose(m_drive, new Pose2d()));
        onTrue(driverControl::resetRotation180, new SetRotation(m_drive, Rotation2d.kPi));

        whileTrue(() -> driverControl.driveToObject(), new SequentialCommandGroup(new DriveToPoseWithProfile(fieldLog, () -> {
            Pose2d x = new Pose2d(FieldConstants.getBargeStation(BargeDestination.CENTER, true), new Rotation2d());
            return new SwerveModel(new Model100(x.getX(),1),new Model100(x.getY(),0),new Model100(x.getRotation().getRadians(),0));
        },m_drive, holonomicController, profile, false), new DriveToPoseWithProfile(fieldLog, () -> {
            Pose2d x = new Pose2d(FieldConstants.getBargeStation(BargeDestination.CENTER, false), new Rotation2d());
            return new SwerveModel(new Model100(x.getX(),0),new Model100(x.getY(),0),new Model100(x.getRotation().getRadians(),0));
        },m_drive, holonomicController, profile)));

        // whileTrue(operatorControl::elevate, new SetElevatorPerpetually(m_elevator,
        // 10));
        whileTrue(driverControl::driveToTag, new ScoreCoral(coralSequence, m_wrist, m_elevator, m_tunnel,
                FieldSector.AB, ReefDestination.LEFT, buttons::scoringPosition, holonomicController, profile, m_drive));

        m_initializer = Executors.newSingleThreadScheduledExecutor();
        m_initializer.schedule(this::initStuff, 0, TimeUnit.SECONDS);

        // This really only does anything when we're sitting idle; when actually
        // running, the gc runs frequently without prodding.
        m_initializer.schedule(System::gc, 3, TimeUnit.SECONDS);
    }

    public void initStuff() {
        Util.println("******** Pre-initializing some expensive things ... ");
        double startS = Takt.actual();

        List<HolonomicPose2d> waypoints = new ArrayList<>();
        waypoints.add(new HolonomicPose2d(
                new Translation2d(),
                Rotation2d.kZero,
                Rotation2d.kZero));
        waypoints.add(new HolonomicPose2d(
                new Translation2d(1, 0),
                Rotation2d.kZero,
                Rotation2d.kZero));
        TrajectoryPlanner m_planner = new TrajectoryPlanner(new TimingConstraintFactory(m_swerveKinodynamics).medium());
        m_planner.restToRest(waypoints);
        m_drive.driveInFieldCoords(new FieldRelativeVelocity(0, 0, 0));
        System.gc();

        double endS = Takt.actual();
        Util.printf("... Initialization ET: %f\n", endS - startS);
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

    private void whileTrue(BooleanSupplier condition1, BooleanSupplier condition2, Command command) {
        Trigger trigger1 = new Trigger(condition1);
        Trigger trigger2 = new Trigger(condition2);

        trigger1.and(trigger2).whileTrue(command);
    }

    private void onTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).onTrue(command);
    }

    public void scheduleAuton() {
        if (m_auton == null)
            return;
        m_auton.schedule();
    }

    public void periodic() {
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
