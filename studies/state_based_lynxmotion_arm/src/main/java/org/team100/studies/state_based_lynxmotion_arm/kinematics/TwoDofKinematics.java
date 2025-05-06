package org.team100.studies.state_based_lynxmotion_arm.kinematics;

import edu.wpi.first.math.geometry.Translation2d;

/** This solves the subproblem of boom and stick angles. */
public class TwoDofKinematics {
    public static class ArmAngles {
        /** proximal radians relative to ground, zero up */
        public final double th1;
        /** distal radians relative to the proximal segment, zero inline */
        public final double th2;

        /**
         * angles in radians, counting out from the grounded joint.
         * 
         * @param th1 proximal relative to ground
         * @param th2 distal relative to proximal
         */
        public ArmAngles(double th1, double th2) {
            this.th1 = th1;
            this.th2 = th2;
        }
    }

    private final double l1;
    private final double l2;

    /**
     * Lengths counting out from the grounded joint. Units here determine units
     * below.
     * 
     * @param l1 proximal
     * @param l2 distal
     */
    public TwoDofKinematics(double l1, double l2) {
        this.l1 = l1;
        this.l2 = l2;
    }

    /**
     * Calculates the position of the arm based on absolute joint angles, counting
     * out from the grounded joint.
     * 
     * @param a absolute angles
     * @return end position
     */
    public Translation2d forward(ArmAngles a) {
        // this is the 2023 way
        // return new Translation2d(
        // l1 * Math.cos(a.th1) + l2 * Math.cos(a.th2),
        // l1 * Math.sin(a.th1) + l2 * Math.sin(a.th2));
        // this is the relative way
        return new Translation2d(
                l1 * Math.cos(a.th1) + l2 * Math.cos(a.th2 + a.th1),
                l1 * Math.sin(a.th1) + l2 * Math.sin(a.th2 + a.th1));
    }

    /**
     * Calculate absolute joint angles given cartesian coords of the end.
     * 
     * It's an application of the law of cosines. It's almost exactly the same as:
     * https://docs.google.com/document/d/135U309CXN29X3Oube1N1DaXPHlo6r-YdnPHMH8NBev8/edit
     * 
     * but theta2 is relative to l1, so there's no th1 in the th2 expression.
     * 
     * @param x
     * @param y
     * @return absolute joint angles, null if unreachable.
     */
    public ArmAngles inverse(Translation2d t) {
        double r = Math.sqrt(t.getX() * t.getX() + t.getY() * t.getY());
        double gamma = Math.atan2(t.getY(), t.getX());
        double beta = Math.acos((r * r + l1 * l1 - l2 * l2) / (2 * r * l1));
        double alpha = Math.acos((l1 * l1 + l2 * l2 - r * r) / (2 * l1 * l2));
        double th1 = gamma - beta;
        // this is how the 2023 arm works:
        // double th2 = Math.PI + th1 - alpha;
        // this is the relative way
        double th2 = Math.PI - alpha;
        if (Double.isNaN(th1) || Double.isNaN(th2))
            return null;
        return new ArmAngles(th1, th2);
    }
}