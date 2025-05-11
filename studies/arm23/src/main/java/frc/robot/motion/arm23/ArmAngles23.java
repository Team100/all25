package org.team100.lib.motion.arm23;

/**
 * Represents the 2DOF serial arm used in 2023.
 * 
 * Absolute angles in radians, counting out from the grounded joint.
 * 
 * @param th1 proximal, absolute, radians
 * @param th2 distal, absolute, radians
 */
public record ArmAngles23(double th1, double th2) {
    @Override
    public String toString() {
        return "ArmAngles [th1=" + th1 + ", th2=" + th2 + "]";
    }
}