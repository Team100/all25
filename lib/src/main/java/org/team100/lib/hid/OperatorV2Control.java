package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.deadband;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a Microsoft Xbox controller, Logitech F310, or similar.
 * 
 * Controls mapping (please keep this in sync with the code below):
 * 
 * <pre>
 * left trigger [0,1]     == 
 * left bumper button     == 
 * left stick x [-1,1]    == 
 * left stick y [-1,1]    == 
 * left stick button      == 
 * dpad/pov angle [0,360] == 
 * "back" button          == 
 * "start" button         == 
 * right stick x [-1,1]   == 
 * right stick y [-1,1]   == 
 * right stick button     == 
 * x button               == 
 * y button               == 
 * a button               == 
 * b button               == 
 * right trigger [0,1]    ==
 * right bumper button    == 
 * </pre>
 * 
 * Do not use stick buttons, they are prone to stray clicks
 */
public class OperatorV2Control implements OperatorControl {
    private final XboxController m_controller;

    public OperatorV2Control() {
        m_controller = new XboxController(1);
    }

    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    @Override
    public boolean getButton1() {
        return m_controller.getAButton();
    }

    @Override
    public boolean getButton2() {
        return m_controller.getBButton();
    }

    @Override
    public boolean getButton3() {
        return m_controller.getXButton();
    }

    @Override
    public boolean getButton4() {
        return m_controller.getYButton();
    }

    @Override
    public boolean getButton5() {
        return m_controller.getLeftBumperButton();
    }

    @Override
    public boolean getButton6() {
        return m_controller.getRightBumperButton();
    }
    
    @Override
    public boolean getButton7() {
        return m_controller.getRightStickButton();
    }

    @Override
    public boolean getButton8() {
        return m_controller.getLeftBumperButton();
    }

    @Override
    public boolean getButton9() {
        return m_controller.getStartButton();
    }

    @Override
    public boolean getButton10() {
        return m_controller.getPOV() == 0;
    }

    @Override
    public boolean getButton11() {
        return m_controller.getPOV() == 90;
    }

    @Override
    public boolean getButton12() {
        return m_controller.getPOV() == 180;
    }

    @Override
    public boolean getButton13() {
        return m_controller.getPOV() == 270;
    }

    @Override
    public double getAxis1() {
        return -deadband(m_controller.getRightY(), 0.2, Double.MAX_VALUE);
    }

    @Override
    public double getAxis2() {
        return -deadband(m_controller.getRightX(), 0.2, Double.MAX_VALUE);
    }

    @Override
    public double getAxis3() {
        return -deadband(m_controller.getLeftY(), 0.2, Double.MAX_VALUE);
    }

    @Override
    public double getAxis4() {
        return -deadband(m_controller.getLeftX(), 0.2, Double.MAX_VALUE);
    }

    @Override
    public double getAxis5() {
        return -deadband(m_controller.getRightTriggerAxis(), 0.2, Double.MAX_VALUE);
    }

    @Override
    public double getAxis6() {
        return -deadband(m_controller.getLeftTriggerAxis(), 0.2, Double.MAX_VALUE);
    }

    
}
