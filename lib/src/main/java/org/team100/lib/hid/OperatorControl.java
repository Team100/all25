package org.team100.lib.hid;

import org.team100.lib.dashboard.Glassy;

/**
 * Represents the HID used by the "operator" role, which typically controls
 * everything other than the drivetrain.
 * 
 * When you change this interface, you'll need to also change the "proxy," don't
 * forget. :-)
 */
public interface OperatorControl extends Glassy {

    default String getHIDName() {
        return "No HID Found!!";
    }

    default double ramp() {
        return 0.0;
    }

    /** placeholder for elevator development */
    default boolean elevate() {
        return false;
    }

    default boolean downavate() {
        return false;
    }

    default boolean intake() {
        return false;
    }

    default boolean outtake() {
        return false;
    }

    default boolean setWrist() {
        return false;
    }

}
