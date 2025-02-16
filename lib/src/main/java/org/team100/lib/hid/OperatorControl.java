package org.team100.lib.hid;

import org.team100.lib.dashboard.Glassy;

/**
 * Represents the HID used by the "operator" role, which typically controls
 * everything other than the drivetrain.
 */
public interface OperatorControl extends Glassy {

    default String getHIDName() {
        return "No HID Found!!";
    }



    // this exists to bind to commands we don't want to run,
    // but we don't want them to rot either.
    default boolean never() {
        return false;
    }




}
