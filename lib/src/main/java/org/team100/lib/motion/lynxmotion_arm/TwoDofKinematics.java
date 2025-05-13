package org.team100.lib.motion.lynxmotion_arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Planar serial arm kinematics: two revolute joints and two links.
 * 
 * Refer to the diagram:
 * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
 */
public class TwoDofKinematics {
    /**
     * Configuration of the joints.
     * 
     * @param q1 Proximal radians, [-pi, pi]. Zero is along the x axis, positive is
     *           counterclockwise.
     * 
     * @param q2 Distal radians, [-pi, pi]. Zero is along the parent link, positive
     *           is counterclockwise. Because the "elbow" is always "up", this angle
     *           is always between pi and 2pi.
     */
    public record TwoDofArmConfig(double q1, double q2) {
        public TwoDofArmConfig(double q1, double q2) {
            this.q1 = MathUtil.angleModulus(q1);
            this.q2 = MathUtil.angleModulus(q2);
        }
    }

    /**
     * Position of each joint.
     */
    public record TwoDofArmPosition(Translation2d p1, Translation2d p2) {
    }

    /** Proximal link length, meters. */
    private final double l1;
    /** Distal link length, meters. */
    private final double l2;

    /**
     * @param l1 Proximal link length, meters.
     * @param l2 Distal link length, meters.
     */
    public TwoDofKinematics(double l1, double l2) {
        this.l1 = l1;
        this.l2 = l2;
    }

    /** Workspace end location based on joint configuration. */
    public TwoDofArmPosition forward(TwoDofArmConfig a) {
        double x1 = l1 * Math.cos(a.q1);
        double y1 = l1 * Math.sin(a.q1);
        double x2 = x1 + l2 * Math.cos(a.q2 + a.q1);
        double y2 = y1 + l2 * Math.sin(a.q2 + a.q1);
        return new TwoDofArmPosition(
                new Translation2d(x1, y1),
                new Translation2d(x2, y2));
    }

    /**
     * Calculate joint configuration given the workspace location of the end.
     * 
     * It's an application of the law of cosines.
     * 
     * Refer to the diagram:
     * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
     */
    public TwoDofArmConfig inverse(Translation2d t) {
        double r = t.getNorm();
        double gamma = Math.atan2(t.getY(), t.getX());
        double beta = Math.acos((r * r + l1 * l1 - l2 * l2) / (2 * r * l1));
        double alpha = Math.acos((l1 * l1 + l2 * l2 - r * r) / (2 * l1 * l2));

        double q1 = gamma + beta;
        double q2 = alpha + Math.PI;

        if (Double.isNaN(q1) || Double.isNaN(q2))
            throw new IllegalArgumentException("invalid two-dof parameters");
        return new TwoDofArmConfig(q1, q2);
    }
}