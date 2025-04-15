package org.team100.frc2025;

import java.io.IOException;

import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.JvmLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.IntLogger;
import org.team100.lib.logging.Logging;
import org.team100.lib.util.Debug;
import org.team100.lib.util.Memo;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot100 implements Debug {
    private final DoubleLogger m_log_ds_MatchTime;
    private final BooleanLogger m_log_ds_AutonomousEnabled;
    private final BooleanLogger m_log_ds_TeleopEnabled;
    private final BooleanLogger m_log_ds_FMSAttached;
    private final IntLogger m_log_key_list_size;
    private final DoubleLogger m_log_voltage;
    private final JvmLogger m_jvmLogger;
    private final DoubleLogger m_log_update;

    private RobotContainer m_robotContainer;

    public Robot() {
        LoggerFactory dsLog = m_robotLogger.child("DriverStation");
        m_log_ds_MatchTime = dsLog.doubleLogger(Level.TRACE, "MatchTime");
        m_log_ds_AutonomousEnabled = dsLog.booleanLogger(Level.TRACE, "AutonomousEnabled");
        m_log_ds_TeleopEnabled = dsLog.booleanLogger(Level.TRACE, "TeleopEnabled");
        m_log_ds_FMSAttached = dsLog.booleanLogger(Level.TRACE, "FMSAttached");
        m_log_key_list_size = m_robotLogger.intLogger(Level.TRACE, "key list size");
        m_log_voltage = m_robotLogger.doubleLogger(Level.TRACE, "voltage");
        m_jvmLogger = new JvmLogger(m_robotLogger);
        m_log_update = m_robotLogger.doubleLogger(Level.COMP, "update time (s)");
        // 4/2/25 Joel added this to try to avoid doing extra work
        setNetworkTablesFlushEnabled(false);
        // CanBridge.runTCP();
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // RobotInit is called once by TimedRobot.startCompetition.
    //

    @Override
    public void robotInit() {
        Util.printf("WPILib Version: %s\n", WPILibVersion.Version); // 2023.2.1
        Util.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        Util.printf("Identity: %s\n", Identity.instance.name());
        RobotController.setBrownoutVoltage(5.5);
        banner();

        // By default, LiveWindow turns off the CommandScheduler in test mode,
        // but we don't want that.
        enableLiveWindowInTest(false);

        // log what the scheduler is doing
        SmartDashboard.putData(CommandScheduler.getInstance());

        try {
            m_robotContainer = new RobotContainer(this);
        } catch (IOException e) {
            throw new IllegalStateException("Robot Container Instantiation Failed", e);
        }

        m_robotContainer.onInit();

        NetworkTableInstance.getDefault().startServer();

        // DataLogManager.start();

        Util.printf("Total Logger Keys: %d\n", Logging.instance().keyCount());

        // This reduces the allocated heap size, not just the used heap size, which
        // means more-frequent and smaller subsequent GC's.
        System.gc();
    }

    /**
     * robotPeriodic is called in the IterativeRobotBase.loopFunc, which is what the
     * TimedRobot runs in the main loop.
     * 
     * This is what should do all the work.
     */
    @Override
    public void robotPeriodic() {
        try {
            // real-time priority for this thread while it's running robotPeriodic.
            // see
            // https://github.com/Mechanical-Advantage/AdvantageKit/blob/a86d21b27034a36d051798e3eaef167076cd302b/template_projects/sources/vision/src/main/java/frc/robot/Robot.java#L90
            // This seems to interfere with CAN at startup
            // Threads.setCurrentThreadPriority(true, 99);

            // Advance the drumbeat.
            Takt.update();

            // Take all the measurements we can, as soon and quickly as possible.
            updateCaches();

            // Run one iteration of the command scheduler.
            CommandScheduler.getInstance().run();

            // Actuate LEDs, do some logging.
            m_robotContainer.periodic();

            m_log_ds_MatchTime.log(DriverStation::getMatchTime);
            m_log_ds_AutonomousEnabled.log(DriverStation::isAutonomousEnabled);
            m_log_ds_TeleopEnabled.log(DriverStation::isTeleopEnabled);
            m_log_ds_FMSAttached.log(DriverStation::isFMSAttached);

            m_jvmLogger.logGarbageCollectors();
            m_jvmLogger.logMemoryPools();
            m_jvmLogger.logMemoryUsage();

            Logging.instance().periodic();

            if (Experiments.instance.enabled(Experiment.FlushOften)) {
                // Util.warn("FLUSHING EVERY LOOP, DO NOT USE IN COMP");
                NetworkTableInstance.getDefault().flush();
            }
        } finally {
            // This thread should have low priority when the main loop isn't running.
            // Threads.setCurrentThreadPriority(false, 0);
        }
    }

    /** Update the measurement caches. */
    private void updateCaches() {
        // Measure how long the update takes prior to the scheduler running.
        double startUpdateS = Takt.actual();

        // Cache instances hold measurements that we want to keep consistent
        // for an entire cycle, but that we want to forget between cycles, so we
        // reset them all here.
        Memo.resetAll();

        // After the clock is advanced, it would be good to make as many observations as
        // possible, so the times of those observations are as close to the interrupt
        // time as possible. This might yield a whole lot of work, though.
        Memo.updateAll();
        double endUpdateS = Takt.actual();
        double updateS = endUpdateS - startUpdateS;
        // this is how long it takes to update all the memos
        m_log_update.log(() -> updateS);
    }

    //////////////////////////////////////////////////////////////////////
    //
    // Exits are called first when mode changes
    //
    @Override
    public void testExit() {
        clearCommands();
    }

    //////////////////////////////////////////////////////////////////////
    //
    // Inits are called on mode change, after exiting the previous mode.
    //

    @Override
    public void autonomousInit() {
        m_robotContainer.onAuto();
        m_robotContainer.scheduleAuton();
    }

    @Override
    public void teleopInit() {
        // this cancels all the default commands, resulting in them being rescheduled
        // immediately, which seems like maybe not great?
        CommandScheduler.getInstance().cancelAll();
        m_robotContainer.cancelAuton();
        m_robotContainer.onTeleop();
    }

    @Override
    public void testInit() {
        clearCommands();
        for (Topic t : NetworkTableInstance.getDefault().getTopics()) {
           // debug("%s\n", t.getName());
        }
        m_robotContainer.scheduleTest();
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // Mode-specific periodics should do nothing, to avoid caching anything.
    //

    @Override
    public void disabledPeriodic() {
        int keyListSize = NetworkTableInstance.getDefault().getTable("Vision").getKeys().size();
        m_log_key_list_size.log(() -> keyListSize);
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
        m_log_voltage.log(RobotController::getBatteryVoltage);
    }

    @Override
    public void testPeriodic() {
        double elevatorControl = NetworkTableInstance.getDefault()
                .getEntry("log/Elevator/Elevator/Starboard/OutboardLinearPositionServo/control (m)/x").getDouble(0);
        double wristControl = NetworkTableInstance.getDefault()
                .getEntry("log/Elevator/Wrist2/OnboardAngularPositionServo/control (rad)/x").getDouble(0);
        double elevatorMeasurement = NetworkTableInstance.getDefault()
                .getEntry("log/Elevator/Elevator/Starboard/OutboardLinearPositionServo/position (m)").getDouble(0);
        double wristMeasurement = NetworkTableInstance.getDefault()
                .getEntry("log/Elevator/Wrist2/OnboardAngularPositionServo/measurement (rad)/x").getDouble(0);
        debug("%12.6f %12.6f %12.6f %12.6f\n",
                elevatorControl / 20,
                wristControl,
                elevatorMeasurement / 20,
                wristMeasurement);

    }

    //////////////////////////////////////////////////////////////////////
    //
    // Simulation init is called once right after RobotInit.
    //

    @Override
    public void simulationInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    //////////////////////////////////////////////////////////////////////
    //
    // Simulation periodic is called after everything else; leave it empty.
    //

    @Override
    public void simulationPeriodic() {
        //
    }

    @Override
    public void close() {
        super.close();
        m_robotContainer.close();
    }

    ///////////////////////////////////////////////////////////////////////

    private void clearCommands() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
    }

    private void banner() {
        StringBuilder b = new StringBuilder();
        b.append("\n");
        b.append("..########.########....###....##.....##.......##.....#####.....#####....\n");
        b.append(".....##....##.........##.##...###...###.....####....##...##...##...##...\n");
        b.append(".....##....##........##...##..####.####.......##...##.....##.##.....##..\n");
        b.append(".....##....######...##.....##.##.###.##.......##...##.....##.##.....##..\n");
        b.append(".....##....##.......#########.##.....##.......##...##.....##.##.....##..\n");
        b.append(".....##....##.......##.....##.##.....##.......##....##...##...##...##...\n");
        b.append(".....##....########.##.....##.##.....##.....######...#####.....#####....\n");
        b.append("\n");
        Util.println(b.toString());

    }

    @Override
    public boolean debug() {
        return true;
    }
}