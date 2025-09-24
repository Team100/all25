package org.team100.frc2025;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

import org.team100.frc2025.Climber.Climber;
import org.team100.frc2025.CommandGroups.GrabAlgaeL2Dumb;
import org.team100.frc2025.CommandGroups.GrabAlgaeL3Dumb;
import org.team100.frc2025.CommandGroups.RunFunnelHandoff;
import org.team100.frc2025.CommandGroups.ScoreBargeSmart;
import org.team100.frc2025.CommandGroups.ScoreSmart.ScoreCoralSmart;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.ElevatorDefaultCommand;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Funnel.FunnelDefault;
import org.team100.frc2025.Funnel.ReleaseFunnel;
import org.team100.frc2025.Swerve.DriveForwardSlowly;
import org.team100.frc2025.Swerve.FieldConstants;
import org.team100.frc2025.Swerve.FieldConstants.ReefPoint;
import org.team100.frc2025.Swerve.ManualWithBargeAssist;
import org.team100.frc2025.Swerve.ManualWithProfiledReefLock;
import org.team100.frc2025.Swerve.Auto.Auton;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.AlgaeGripDefaultCommand;
import org.team100.frc2025.Wrist.AlgaeOuttakeGroup;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.frc2025.Wrist.WristDefaultCommand;
import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.coherence.Takt;
import org.team100.lib.commands.drivetrain.ResetPose;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.commands.drivetrain.manual.DriveManually;
import org.team100.lib.commands.drivetrain.manual.DriveManuallySimple;
import org.team100.lib.commands.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.commands.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.commands.drivetrain.manual.ManualWithFullStateHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithProfiledHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithTargetLock;
import org.team100.lib.commands.drivetrain.manual.SimpleManualModuleStates;
import org.team100.lib.config.Camera;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.drivetrain.FullStateSwerveController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.examples.semiauto.FloorPickSetup;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.gyro.Gyro;
import org.team100.lib.gyro.GyroFactory;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.hid.OperatorControlProxy;
import org.team100.lib.hid.ThirdControl;
import org.team100.lib.hid.ThirdControlProxy;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.localization.FreshSwerveEstimate;
import org.team100.lib.localization.NudgingVisionUpdater;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SimulatedTagDetector;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LevelPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.targeting.SimulatedTargetWriter;
import org.team100.lib.targeting.Targets;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Try to keep this container clean; if there's something you want to keep but
 * don't need right now, cut and paste it into {@link RobotContainerParkingLot}.
 */
public class RobotContainer {
    // for background on drive current limits:
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    // https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
    // https://docs.google.com/document/d/10uXdmu62AFxyolmwtDY8_9UNnci7eVcev4Y64ZS0Aqk
    // https://github.com/frc1678/C2024-Public/blob/17e78272e65a6ce4f87c00a3514c79f787439ca1/src/main/java/com/team1678/frc2024/Constants.java#L195
    // 2/26/25: Joel updated the supply limit to 90A, see 1678 code above. This is
    // essentially unlimited, so you'll need to run some other kind of limiter (e.g.
    // acceleration) to keep from browning out.
    private static final double DRIVE_SUPPLY_LIMIT = 90;
    private static final double DRIVE_STATOR_LIMIT = 110;

    private final SwerveModuleCollection m_modules;
    private final Command m_auton;
    /**
     * This runs on enable for test, so i can run stuff in simulation without
     * fiddling with buttons. But it will also run in real life, so be careful.
     */
    private final Command m_test;

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

    private final Runnable m_simulatedTagDetector;
    private final Runnable m_targetSimulator;
    private final Runnable m_combinedViz;
    private final Targets m_targets;

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
        final LoggerFactory driveLog = logger.name("Drive");
        final LoggerFactory comLog = logger.name("Commands");
        final LoggerFactory elevatorLog = logger.name("Elevator");
        final LoggerFactory coralSequence = logger.name("Coral Sequence");
        final LoggerFactory autoSequence = logger.name("Auto Sequence");

        final TrajectoryVisualization viz = new TrajectoryVisualization(fieldLogger);
        final DriverControl driverControl = new DriverControlProxy(logger, async);
        final OperatorControl operatorControl = new OperatorControlProxy(async);
        final ThirdControl buttons = new ThirdControlProxy(async);

        if (Identity.instance.equals(Identity.COMP_BOT)) {
            m_elevator = new Elevator(elevatorLog, 11, 19);
            m_wrist = new Wrist2(elevatorLog, 9, m_elevator::getSetpointAcceleration);
            m_tunnel = new CoralTunnel(elevatorLog, 3, 25);
            m_funnel = new Funnel(logger, 23, 14);
            m_grip = new AlgaeGrip(logger, m_tunnel);
            m_climber = new Climber(logger, 15);
            m_swerveKinodynamics = SwerveKinodynamicsFactory.get();
        } else {
            m_swerveKinodynamics = SwerveKinodynamicsFactory.get();
            m_tunnel = new CoralTunnel(elevatorLog, 3, 25);
            m_grip = new AlgaeGrip(logger, m_tunnel);
            m_elevator = new Elevator(elevatorLog, 2, 19);
            m_wrist = new Wrist2(elevatorLog, 9, m_elevator::getSetpointAcceleration);
            m_funnel = new Funnel(logger, 23, 14);
            m_climber = new Climber(logger, 18);
        }

        m_combinedViz = new CombinedVisualization(m_elevator, m_wrist);

        m_test = new Coordinated(m_elevator, m_wrist);

        m_modules = SwerveModuleCollection.get(
                driveLog,
                DRIVE_SUPPLY_LIMIT,
                DRIVE_STATOR_LIMIT,
                m_swerveKinodynamics);

        final Gyro gyro = GyroFactory.get(
                driveLog,
                m_swerveKinodynamics,
                m_modules);

        // ignores the rotation derived from vision.
        final SwerveHistory history = new SwerveHistory(
                m_swerveKinodynamics,
                gyro.getYawNWU(),
                m_modules.positions(),
                Pose2d.kZero,
                Takt.get());

        final OdometryUpdater odometryUpdater = new OdometryUpdater(
                m_swerveKinodynamics, gyro, history, m_modules::positions);
        odometryUpdater.reset(Pose2d.kZero);
        final NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(history, odometryUpdater);

        final AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        final AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                driveLog,
                layout,
                history,
                visionUpdater);

        m_targets = new Targets(driveLog, fieldLog, history);

        final FreshSwerveEstimate estimate = new FreshSwerveEstimate(
                localizer, odometryUpdater, history);

        final SwerveLocal swerveLocal = new SwerveLocal(
                driveLog,
                m_swerveKinodynamics,
                m_modules);

        SwerveLimiter limiter = new SwerveLimiter(driveLog, m_swerveKinodynamics, RobotController::getBatteryVoltage);

        m_drive = new SwerveDriveSubsystem(
                fieldLogger,
                driveLog,
                m_swerveKinodynamics,
                odometryUpdater,
                estimate,
                swerveLocal,
                limiter);

        m_leds = new LEDIndicator(0, localizer::getPoseAgeSec);

        if (RobotBase.isReal()) {
            // Real robots get an empty simulated tag detector.
            m_simulatedTagDetector = () -> {
            };
            m_targetSimulator = () -> {
            };
        } else {
            // In simulation, we want the real simulated tag detector.
            SimulatedTagDetector sim = new SimulatedTagDetector(
                    List.of(
                            Camera.SWERVE_LEFT,
                            Camera.SWERVE_RIGHT,
                            Camera.FUNNEL,
                            Camera.CORAL_LEFT,
                            Camera.CORAL_RIGHT),
                    layout,
                    history);
            m_simulatedTagDetector = sim::periodic;
            // for now, one target, near the corner.
            SimulatedTargetWriter tsim = new SimulatedTargetWriter(
                    List.of(Camera.SWERVE_LEFT,
                            Camera.SWERVE_RIGHT,
                            Camera.FUNNEL,
                            Camera.CORAL_LEFT,
                            Camera.CORAL_RIGHT),
                    history,
                    new Translation2d[] {
                            FieldConstants.CoralMark.LEFT.value,
                            FieldConstants.CoralMark.CENTER.value,
                            FieldConstants.CoralMark.RIGHT.value });
            m_targetSimulator = tsim::update;
        }

        ///////////////////////////
        //
        // DRIVE CONTROLLERS
        //

        final SwerveController holonomicController = SwerveControllerFactory.byIdentity(comLog);

        final DriveManually driveManually = new DriveManually(
                driverControl::velocity,
                localizer::setHeedRadiusM,
                m_drive);
        final LoggerFactory manLog = comLog.type(driveManually);

        final Feedback100 thetaFeedback = new PIDFeedback(
                manLog, 3.2, 0, 0, true, 0.05, 1);

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

        driveManually.register("BARGE ASSIST", false,
                new ManualWithBargeAssist(
                        manLog,
                        m_swerveKinodynamics,
                        driverControl::desiredRotation,
                        thetaFeedback,
                        m_drive));

        driveManually.register("REEF LOCK", false,
                new ManualWithProfiledReefLock(
                        manLog,
                        m_swerveKinodynamics,
                        driverControl::useReefLock,
                        thetaFeedback,
                        m_drive,
                        buttons::red1));

        // DEFAULT COMMANDS

        DriveManuallySimple driveDefault = new DriveManuallySimple(
                driverControl::velocity,
                localizer::setHeedRadiusM,
                m_drive,
                new ManualWithProfiledReefLock(
                        manLog, m_swerveKinodynamics, driverControl::useReefLock, thetaFeedback,
                        m_drive, buttons::red1),
                new ManualWithBargeAssist(
                        manLog, m_swerveKinodynamics, driverControl::desiredRotation, thetaFeedback,
                        m_drive),
                driverControl::driveWithBargeAssist);

        m_drive.setDefaultCommand(driveDefault);
        m_climber.setDefaultCommand(m_climber.stop());
        m_elevator.setDefaultCommand(new ElevatorDefaultCommand(elevatorLog, m_elevator, m_wrist, m_grip, m_drive));
        m_wrist.setDefaultCommand(new WristDefaultCommand(elevatorLog, m_wrist, m_elevator, m_grip, m_drive));
        m_grip.setDefaultCommand(new AlgaeGripDefaultCommand(m_grip));
        m_funnel.setDefaultCommand(new FunnelDefault(m_funnel));

        // DRIVER BUTTONS
        final HolonomicProfile profile = HolonomicProfile.get(driveLog, m_swerveKinodynamics, 1, 0.5, 1, 0.2);

        final HolonomicProfile autoProfile = HolonomicProfile.currentLimitedExponential(3.5, 7, 20,
                m_swerveKinodynamics.getMaxAngleSpeedRad_S(), m_swerveKinodynamics.getMaxAngleAccelRad_S2(), 20);

        FullStateSwerveController autoController = SwerveControllerFactory.auto2025LooseTolerance(autoSequence);

        m_auton = new Auton(logger, m_wrist, m_elevator, m_funnel, m_tunnel, m_grip, autoController,
                autoProfile, m_drive, localizer::setHeedRadiusM, m_swerveKinodynamics, viz).left();

        whileTrue(driverControl::test,
                new Auton(logger, m_wrist, m_elevator, m_funnel, m_tunnel, m_grip, autoController,
                        autoProfile, m_drive, localizer::setHeedRadiusM, m_swerveKinodynamics, viz).right());

        // Driver/Operator Buttons
        onTrue(driverControl::resetRotation0, new ResetPose(m_drive, new Pose2d()));
        onTrue(driverControl::resetRotation180, new SetRotation(m_drive, Rotation2d.kPi));
        whileTrue(driverControl::feedFunnel,
                RunFunnelHandoff.get(comLog, m_elevator, m_wrist, m_funnel, m_tunnel, m_grip));
        whileTrue(driverControl::climb,
                new ParallelCommandGroup(
                        m_climber.setPosition(0.53), new DriveForwardSlowly(m_drive)));

        whileTrue(driverControl::driveToTag, buttons::a,
                ScoreCoralSmart.get(coralSequence, m_wrist, m_elevator, m_tunnel,
                        buttons::scoringPosition, holonomicController, profile,
                        m_drive, localizer::setHeedRadiusM, ReefPoint.A));

        whileTrue(driverControl::driveToTag, buttons::b,
                ScoreCoralSmart.get(coralSequence, m_wrist, m_elevator, m_tunnel,
                        buttons::scoringPosition, holonomicController, profile,
                        m_drive, localizer::setHeedRadiusM, ReefPoint.B));

        whileTrue(driverControl::driveToTag, buttons::c,
                ScoreCoralSmart.get(coralSequence, m_wrist, m_elevator, m_tunnel,
                        buttons::scoringPosition, holonomicController, profile,
                        m_drive, localizer::setHeedRadiusM, ReefPoint.C));

        whileTrue(driverControl::driveToTag, buttons::d,
                ScoreCoralSmart.get(coralSequence, m_wrist, m_elevator, m_tunnel,
                        buttons::scoringPosition, holonomicController, profile,
                        m_drive, localizer::setHeedRadiusM, ReefPoint.D));

        whileTrue(driverControl::driveToTag, buttons::e,
                ScoreCoralSmart.get(coralSequence, m_wrist, m_elevator, m_tunnel,
                        buttons::scoringPosition, holonomicController, profile,
                        m_drive, localizer::setHeedRadiusM, ReefPoint.E));

        whileTrue(driverControl::driveToTag, buttons::f,
                ScoreCoralSmart.get(coralSequence, m_wrist, m_elevator, m_tunnel,
                        buttons::scoringPosition, holonomicController, profile,
                        m_drive, localizer::setHeedRadiusM, ReefPoint.F));

        whileTrue(driverControl::driveToTag, buttons::g,
                ScoreCoralSmart.get(coralSequence, m_wrist, m_elevator, m_tunnel,
                        buttons::scoringPosition, holonomicController, profile,
                        m_drive, localizer::setHeedRadiusM, ReefPoint.G));

        whileTrue(driverControl::driveToTag, buttons::h,
                ScoreCoralSmart.get(coralSequence, m_wrist, m_elevator, m_tunnel,
                        buttons::scoringPosition, holonomicController, profile,
                        m_drive, localizer::setHeedRadiusM, ReefPoint.H));

        whileTrue(driverControl::driveToTag, buttons::i,
                ScoreCoralSmart.get(coralSequence, m_wrist, m_elevator, m_tunnel,
                        buttons::scoringPosition, holonomicController, profile,
                        m_drive, localizer::setHeedRadiusM, ReefPoint.I));

        whileTrue(driverControl::driveToTag, buttons::j,
                ScoreCoralSmart.get(coralSequence, m_wrist, m_elevator, m_tunnel,
                        buttons::scoringPosition, holonomicController, profile,
                        m_drive, localizer::setHeedRadiusM, ReefPoint.J));

        whileTrue(driverControl::driveToTag, buttons::k,
                ScoreCoralSmart.get(coralSequence, m_wrist, m_elevator, m_tunnel,
                        buttons::scoringPosition, holonomicController, profile,
                        m_drive, localizer::setHeedRadiusM, ReefPoint.K));

        whileTrue(driverControl::driveToTag, buttons::l,
                ScoreCoralSmart.get(coralSequence, m_wrist, m_elevator, m_tunnel,
                        buttons::scoringPosition, holonomicController, profile,
                        m_drive, localizer::setHeedRadiusM, ReefPoint.L));

        whileTrue(buttons::ab, GrabAlgaeL3Dumb.get(m_wrist, m_elevator, m_grip));
        whileTrue(buttons::cd, GrabAlgaeL2Dumb.get(m_wrist, m_elevator, m_grip));
        whileTrue(buttons::ef, GrabAlgaeL3Dumb.get(m_wrist, m_elevator, m_grip));
        whileTrue(buttons::gh, GrabAlgaeL2Dumb.get(m_wrist, m_elevator, m_grip));
        whileTrue(buttons::ij, GrabAlgaeL3Dumb.get(m_wrist, m_elevator, m_grip));
        whileTrue(buttons::kl, GrabAlgaeL2Dumb.get(m_wrist, m_elevator, m_grip));

        whileTrue(buttons::red2,
                AlgaeOuttakeGroup.get(m_grip, m_wrist, m_elevator));
        whileTrue(buttons::red3,
                new ScoreBargeSmart(m_elevator, m_wrist, m_grip, buttons::red4));
        whileTrue(buttons::barge,
                ReleaseFunnel.get(logger, m_funnel, m_climber));

        whileTrue(operatorControl::activateManualClimb,
                m_climber.manual(operatorControl::manualClimbSpeed));

        // this is for developing autopick.
        new FloorPickSetup(
                fieldLog, driverControl, m_drive, m_targets, 
                SwerveControllerFactory.pick(driveLog), autoProfile);

        m_initializer = Executors.newSingleThreadScheduledExecutor();
        m_initializer.schedule(this::initStuff, 0, TimeUnit.SECONDS);

        // This really only does anything when we're sitting idle; when actually
        // running, the gc runs frequently without prodding.
        m_initializer.schedule(System::gc, 3, TimeUnit.SECONDS);
    }

    public void initStuff() {
        Util.println("\n******** Pre-initializing some expensive things ... ");
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
        Util.printf("\n... Initialization ET: %f\n", endS - startS);
    }

    public void onTeleop() {
        // TODO: remove
    }

    public void onInit() {
        m_drive.resetPose(new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d(Math.PI)));
    }

    public void onAuto() {
        // TODO: remove
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

    public void scheduleTest() {
        if (m_test == null)
            return;
        m_test.schedule();
    }

    public void periodic() {
        // publish the simulated tag sightings.
        m_simulatedTagDetector.run();
        // publish simulated target sightings
        m_targetSimulator.run();
        // show the closest target on field2d
        m_targets.periodic();
        m_leds.periodic();
        m_combinedViz.run();
    }

    public void cancelAuton() {
        // TODO: remove
    }

    // this keeps the tests from conflicting via the use of simulated HAL ports.
    public void close() {
        m_modules.close();
        m_leds.close();
        m_elevator.close();
    }
}
