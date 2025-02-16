package org.team100.lib.hid;

import org.team100.lib.async.Async;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.StringLogger;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Checks periodically for changes in the HID connected to port 0, and changes
 * the driver control implementation to match.
 */
public class DriverControlProxy implements DriverControl {
    private static class NoDriverControl implements DriverControl {
    }

    private static final int kPort = 0;
    private static final double kFreq = 1;

    private String m_name;
    private DriverControl m_driverControl;
    /** Used by the private factory method below. */
    private final LoggerFactory m_logger;
    private final StringLogger m_hidLogger;

    /**
     * The async is just to scan for control updates, maybe don't use a whole thread
     * for it.
     */
    public DriverControlProxy(LoggerFactory parent, Async async) {
        m_logger = parent.child(this);
        m_hidLogger = m_logger.stringLogger(Level.COMP, "HID");
        refresh();
        async.addPeriodic(this::refresh, kFreq, "DriverControlProxy");
    }

    public void refresh() {
        // name is blank if not connected
        String name = DriverStation.getJoystickName(kPort);
        name = name.trim();
        if (name.equals(m_name)) {
            return;
        }
        m_name = name;
        m_driverControl = getDriverControl(m_logger, name);

        Util.printf("*** CONTROL UPDATE\n");
        Util.printf("*** Driver HID: %s Control: %s\n",
                m_driverControl.getHIDName(),
                m_driverControl.getClass().getSimpleName());
        m_hidLogger.log(() -> m_driverControl.getClass().getSimpleName());
    }

    private static DriverControl getDriverControl(LoggerFactory parent, String name) {
        Util.printf("Trying to match HID name '%s'\n", name);
        if (name.contains("F310")) {
            return new DriverXboxControl(parent);
        }
        if (name.contains("Xbox")) {
            return new DriverXboxControl(parent);
        }
        if (name.contains("VKBsim")) {
            return new VKBJoystick();
        }
        if (name.startsWith("Logitech Extreme")) {
            return new LogitechExtremeJoystick();
        }
        if (name.contains("8Bit")) {
            return new DriverXboxControl(parent);
        }
        if (name.startsWith("GP Controller")) {
            return new RealFlight(parent);
        }
        if (name.equals("Team 100 Pilot")) {
            return new Pilot();
        }
        if (name.contains("Keyboard")) {
            return new SimulatedJoystick();
        }
        if (name.contains("InterLinkDX")) {
            return new InterLinkDX();
        }
        Util.warn("No HID Match!");
        return new NoDriverControl();
    }

    @Override
    public String getHIDName() {
        return m_driverControl.getHIDName();
    }

    @Override
    public double shooterPivot() {
        return m_driverControl.shooterPivot();
    }

    @Override
    public boolean shoot() {
        return m_driverControl.shoot();
    }

    @Override
    public Velocity velocity() {
        return m_driverControl.velocity();
    }

    @Override
    public Rotation2d desiredRotation() {
        return m_driverControl.desiredRotation();
    }

    @Override
    public boolean fullCycle() {
        return m_driverControl.fullCycle();
    }

    @Override
    public boolean driveToObject() {
        return m_driverControl.driveToObject();
    }

    @Override
    public boolean driveToAmp() {
        return m_driverControl.driveToAmp();
    }

    @Override
    public Translation2d target() {
        return m_driverControl.target();
    }

    @Override
    public boolean trigger() {
        return m_driverControl.trigger();
    }

    @Override
    public boolean resetRotation0() {
        return m_driverControl.resetRotation0();
    }

    @Override
    public boolean resetRotation180() {
        return m_driverControl.resetRotation180();
    }

    @Override
    public boolean defense() {
        return m_driverControl.defense();
    }

    @Override
    public boolean driveWithFancyTrajec() {
        return m_driverControl.driveWithFancyTrajec();
    }

    @Override
    public boolean actualCircle() {
        return m_driverControl.actualCircle();
    }

    @Override
    public boolean never() {
        return m_driverControl.never();
    }

    @Override
    public boolean test() {
        return m_driverControl.test();
    }

    @Override
    public boolean shooterLock() {
        return m_driverControl.shooterLock();
    }

    @Override
    public boolean button4() {
        return m_driverControl.button4();
    }

    @Override
    public boolean button5() {
        return m_driverControl.button5();
    }

    @Override
    public boolean ampLock() {
        return m_driverControl.ampLock();
    }

}
