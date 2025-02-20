package org.team100.lib.hid;

import org.team100.lib.async.Async;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Checks periodically for changes in the HID connected to port 1, and changes
 * the operator control implementation to match.
 */
public class OperatorControlProxy implements OperatorControl {
    private static class NoOperatorControl implements OperatorControl {
    }

    private static final int kPort = 1;
    private static final double kFreq = 1;

    private String m_name;
    private OperatorControl m_operatorControl;

    /**
     * The async is just to scan for control updates, maybe don't use a whole thread
     * for it.
     */
    public OperatorControlProxy(Async async) {
        refresh();
        async.addPeriodic(this::refresh, kFreq, "OperatorControlProxy");
    }

    public void refresh() {
        // name is blank if not connected
        String name = DriverStation.getJoystickName(kPort);
        name = name.trim();
        if (name.equals(m_name))
            return;
        m_name = name;
        m_operatorControl = getOperatorControl(name);

        Util.printf("*** CONTROL UPDATE\n");
        Util.printf("*** Operator HID: %s Control: %s\n",
                m_operatorControl.getHIDName(),
                m_operatorControl.getClass().getSimpleName());
    }

    private static OperatorControl getOperatorControl(String name) {
        if (name.contains("F310")) {
            return new OperatorV2Control();
        }
        if (name.contains("8Bit")) {
            return new OperatorV2Control();
        }
        if (name.contains("Xbox")) {
            return new OperatorV2Control();
        }
        if (name.startsWith("MSP430")) {
            // the old button board
            return new NoOperatorControl();
        }
        if (name.contains("Keyboard")) {
            return new OperatorV2Control();
        }
        return new NoOperatorControl();
    }

    @Override
    public String getHIDName() {
        return m_operatorControl.getHIDName();
    }

    @Override
    public Double ramp() {
        return m_operatorControl.ramp();
    }

    @Override
    public boolean never() {
        return m_operatorControl.never();
    }

    @Override
    public boolean elevate() {
        return m_operatorControl.elevate();
    }

    @Override
    public boolean downavate() {
        return m_operatorControl.downavate();
    }

}
