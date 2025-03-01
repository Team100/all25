package org.team100.lib.hid;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a Microsoft Xbox controller, Logitech F310, or similar.
 * 
 * Controls mapping (please keep this in sync with the code below):
 * 
 * <pre>
 * left trigger [0,1]     == 
 * left bumper button     == amp arm up
 * left stick x [-1,1]    == 
 * left stick y [-1,1]    == left climber duty cycle
 * left stick button      == feed to amp
 * dpad/pov angle [0,360] == climber position (0=up, 180=down)
 * "back" button          == home climber
 * "start" button         == test shoot (and selftest enable)
 * right stick x [-1,1]   == 
 * right stick y [-1,1]   == right climber duty cycle
 * right stick button     == outtake from amp
 * x button               == intake
 * y button               == 
 * a button               == ramp shooter speed and angle
 * b button               == outtake
 * right trigger [0,1]    ==
 * right bumper button    == feed to shoot
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
    public Double ramp() {
        return m_controller.getLeftX();
    }
 
    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    @Override
    public boolean elevate() {
        return m_controller.getXButton();
    }

    @Override
    public boolean downavate() {
        return m_controller.getYButton();
    }

    @Override
    public boolean intake() {
        return m_controller.getAButton();
    }

    @Override
    public boolean outtake() {
        return m_controller.getBButton();
    }

    @Override
    public boolean setWrist() {
        return m_controller.getLeftBumperButton();
    }
}
