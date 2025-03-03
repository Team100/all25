package org.team100.frc2025;

/** Computes vertical center of gravity. */
public class VCG {
    public static double vcg(double elevatorPositionM) {
        // robot mass is about 50 kg
        // elevator mass is about 20 kg
        return 0.6 * 0.3 + 0.4 * elevatorPositionM / 2;
    }

}
