package org.team100.frc2025;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.indicator.Alerts;
import org.team100.lib.indicator.SolidIndicator;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.subsystems.tank.TankDrive;
import org.team100.lib.subsystems.tank.TankDriveFactory;
import org.team100.lib.subsystems.tank.commands.TankManual;
import org.team100.lib.util.Banner;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot100 {
    private static final double MAX_SPEED_M_S = 3.0;
    private static final double MAX_OMEGA_RAD_S = 3.0;
    private final TankDrive m_drive;
    private final TrajectoryVisualization m_trajectoryViz;
    private final Autons m_autons;
    private final SolidIndicator m_indicator;
    private final Alerts m_alerts;
    private final Alert m_noStartingPosition;
    private final Alert m_mismatchedAlliance;

    public Robot() {
        Banner.printBanner();
        DriverStation.silenceJoystickConnectionWarning(true);
        Experiments.instance.show();
        SmartDashboard.putData(CommandScheduler.getInstance());
        m_indicator = new SolidIndicator(new RoboRioChannel(0), 40);
        m_alerts = new Alerts();
        m_noStartingPosition = m_alerts.add("No starting position!", AlertType.kWarning);
        m_mismatchedAlliance = m_alerts.add("Wrong Alliance!", AlertType.kWarning);
        m_indicator.state(() -> m_alerts.any() ? Color.kRed : Color.kGreen);
        Logging logging = Logging.instance();
        LoggerFactory fieldLogger = logging.fieldLogger;
        DriverXboxControl driverControl = new DriverXboxControl(0);
        LoggerFactory logger = logging.rootLogger;
        m_drive = TankDriveFactory.make(
                fieldLogger,
                logger,
                60, // stator current limit (a)
                new CanId(6), // left
                new CanId(5), // right
                0.4, // track width
                6.0, // gear ratio
                0.15); // wheel dia (m)
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.tank(logger);
        SwerveLimiter limiter = new SwerveLimiter(
                logger,
                kinodynamics,
                RobotController::getBatteryVoltage);
        TankManual manual = new TankManual(
                logger,
                () -> -1.0 * driverControl.rightY(),
                () -> -1.0 * driverControl.rightX(),
                MAX_SPEED_M_S,
                MAX_OMEGA_RAD_S,
                limiter,
                m_drive);
        m_drive.setDefaultCommand(
                manual);
        m_trajectoryViz = new TrajectoryVisualization(fieldLogger);

        m_autons = new Autons(logger, m_drive, m_trajectoryViz);


    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        if (Experiments.instance.enabled(Experiment.FlushOften)) {
            NetworkTableInstance.getDefault().flush();
        }
    }

    @Override
    public void autonomousInit() {
        // choose a command at auton init time
        AnnotatedCommand cmd = m_autons.get();
        if (cmd == null)
            return;
        Command auton = cmd.command();
        if (auton == null)
            return;
        auton.schedule();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {
        checkStart();
        checkAlliance();
    }

    private void checkStart() {
        AnnotatedCommand cmd = m_autons.get();
        if (cmd == null)
            return;
        Pose2d start = cmd.start();
        if (start == null) {
            m_noStartingPosition.set(true);
        } else {
            m_noStartingPosition.set(false);
            m_drive.setPose(start);
        }
    }

    private void checkAlliance() {
        AnnotatedCommand cmd = m_autons.get();
        if (cmd == null)
            return;
        Alliance alliance = cmd.alliance();
        if (alliance == null) {
            // works for either
            return;
        }
        Alliance dsAlliance = DriverStation.getAlliance().orElse(null);
        if (dsAlliance == null) {
            // not set yet
            return;
        }
        if (alliance != dsAlliance) {
            m_mismatchedAlliance.set(true);
        } else {
            m_mismatchedAlliance.set(false);
        }
    }

}