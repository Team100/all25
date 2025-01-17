package org.team100.lib.hid;

import java.util.HashMap;
import java.util.Map;

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


    public enum GamepadControls {
        A(1),
        B(2),
        X(3),
        Y(4),
        LeftBumper(5),
        RightBumper(6),
        RightStickButton(7),
        LeftStickButton(8),
        Start(9),
        POVUp(10),
        POVRight(11),
        POVDown(12),
        POVLeft(13),
        RightYAxis(1),
        RightXAxis(2),
        LeftYAxis(3),
        LeftXAxis(4),
        RightTriggerAxis(5),
        LeftTriggerAxis(6);
       
    
        private final int value;
    
        GamepadControls(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }
    }

    private static final int kPort = 1;
    private static final double kFreq = 1;

    private String m_name;
    private OperatorControl m_operatorControl;

    private final Map<String, Integer> genericButtons = new HashMap<>();
    private final Map<String, Integer> genericAxis = new HashMap<>();

    private final Map<String, GamepadControls> gamepadButtons = new HashMap<>();
    private final Map<String, GamepadControls> gamepadAxis = new HashMap<>();

    

    /**
     * The async is just to scan for control updates, maybe don't use a whole thread
     * for it.
     */
    public OperatorControlProxy(Async async) {
        populateGenericControls();
        populateGamePadControls();

        refresh();
        async.addPeriodic(this::refresh, kFreq, "OperatorControlProxy");
    }



    public void populateGenericControls() {
        // Map each action name to its corresponding method
        genericAxis.put("climb", 1);

    }

    public void populateGamePadControls() {
        // Map each action name to its corresponding method
        gamepadAxis.put("climb", GamepadControls.RightYAxis);

    }

    public boolean retrieveButton(String action){
        if(m_name == "F310" || m_name == "Xbox"){
            return getButton(gamepadButtons.get(action).getValue());
        }
        return getButton(genericButtons.get(action));
    }

    public double retrieveAxis(String action){
        if(m_name == "F310" || m_name == "Xbox"){
            return getAxis(gamepadAxis.get(action).getValue());
        }
        return getAxis(genericAxis.get(action));
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

    //MISC
    @Override
    public String getHIDName() {
        return m_operatorControl.getHIDName();
    }

    
    //Generic Buttons
    public boolean getButton(int buttonNumber) {
        switch (buttonNumber) {
            case 1: return m_operatorControl.getButton1();
            case 2: return m_operatorControl.getButton2();
            case 3: return m_operatorControl.getButton3();
            case 4: return m_operatorControl.getButton4();
            case 5: return m_operatorControl.getButton5();
            case 6: return m_operatorControl.getButton6();
            case 7: return m_operatorControl.getButton7();
            case 8: return m_operatorControl.getButton8();
            case 9: return m_operatorControl.getButton9();
            case 10: return m_operatorControl.getButton10();
            case 11: return m_operatorControl.getButton11();
            case 12: return m_operatorControl.getButton12();
            case 13: return m_operatorControl.getButton13();
            default: throw new IllegalArgumentException("Invalid button number: " + buttonNumber);
        }
    }

    
    //Generic Axis
    public double getAxis(int axisNumber) {
        switch (axisNumber) {
            case 1: return m_operatorControl.getAxis1();
            case 2: return m_operatorControl.getAxis2();
            case 3: return m_operatorControl.getAxis3();
            case 4: return m_operatorControl.getAxis4();
            case 5: return m_operatorControl.getAxis5();
            default: throw new IllegalArgumentException("Invalid button number: " + axisNumber);
        }
    }

    //ACTIONS
    public double climb(){
        return retrieveAxis("climb");
    }   

    



}
