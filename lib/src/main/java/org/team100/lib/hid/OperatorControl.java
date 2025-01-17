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

    default boolean getButton1() {
        return false;
    }

    default boolean getButton2() {
        return false;
    }

    default boolean getButton3() {
        return false;
    }

    default boolean getButton4() {
        return false;
    }

    default boolean getButton5() {
        return false;
    }

    default boolean getButton6() {
        return false;
    }

    default boolean getButton7() {
        return false;
    }

    default boolean getButton8() {
        return false;
    }

    default boolean getButton9() {
        return false;
    }

    default boolean getButton10() {
        return false;
    }

    default boolean getButton11() {
        return false;
    }

    default boolean getButton12() {
        return false;
    }

    default boolean getButton13() {
        return false;
    }

    default boolean getButton14() {
        return false;
    }
    
    default boolean getButton15() {
        return false;
    }

    default double getAxis1() {
        return 0.0;
    }

    default double getAxis2() {
        return 0.0;
    }

    default double getAxis3() {
        return 0.0;
    }

    default double getAxis4() {
        return 0.0;
    }

    default double getAxis5() {
        return 0.0;
    }

    default double getAxis6() {
        return 0.0;
    }

    default double getAxis7() {
        return 0.0;
    }

    default double getAxis8() {
        return 0.0;
    }

    default double getAxis9() {
        return 0.0;
    }
    
}
