package org.team100.lib.motion.arm;

/**
 * Represents a 2DOF serial arm.
 * 
 * Absolute angles in radians, counting out from the grounded joint.
 * 
 * @param th1 proximal, absolute, radians
 * @param th2 distal, absolute, radians
 */
public record ArmAngles(double th1, double th2) {
    @Override
    public String toString() {
        return "ArmAngles [th1=" + th1 + ", th2=" + th2 + "]";
    }
}