package org.team100.lib.hid;

/**
 * Represents a third control beyond the driver and operator, for example, knobs
 * for tuning, or a MIDI keyboard.
 */
public interface ThirdControl {

    default String getHIDName() {
        return "No HID Found!!";
    }

    default boolean red1() {
        return false;
    }

    default boolean red2() {
        return false;
    }

    default boolean red3() {
        return false;
    }

    default boolean red4() {
        return false;
    }

    default boolean barge() {
        return false;
    }

    // Coral Scoring Levels

    default boolean l1() {
        return false;
    }

    default boolean l2() {
        return false;
    }

    default boolean l3() {
        return false;
    }

    default boolean l4() {
        return false;
    }

    // Coral Scoring

    default boolean a() {
        return false;
    }

    default boolean b() {
        return false;
    }

    default boolean c() {
        return false;
    }

    default boolean d() {
        return false;
    }

    default boolean e() {
        return false;
    }

    default boolean f() {
        return false;
    }

    default boolean g() {
        return false;
    }

    default boolean h() {
        return false;
    }

    default boolean i() {
        return false;
    }

    default boolean j() {
        return false;
    }

    default boolean k() {
        return false;
    }

    default boolean l() {
        return false;
    }

    default boolean ab() {
        return false;
    }

    default boolean cd() {
        return false;
    }

    default boolean ef() {
        return false;
    }

    default boolean gh() {
        return false;
    }

    default boolean ij() {
        return false;
    }

    default boolean kl() {
        return false;
    }

}
