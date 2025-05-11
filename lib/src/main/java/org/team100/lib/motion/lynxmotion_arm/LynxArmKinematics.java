package org.team100.lib.motion.lynxmotion_arm;

import org.team100.lib.motion.lynxmotion_arm.TwoDofKinematics.ArmAngles;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The six-dof arm is used as a four-dof here:
 * swing, boom, stick, wrist.
 * the twist is always level, the grip is always closed.
 */
public class LynxArmKinematics {
    private final LynxArmAngles.Factory m_factory;
    public final double m_boomLength;
    public final double m_stickLength;
    public final double m_wristLength;

    private final TwoDofKinematics twodof;

    public LynxArmKinematics(
            LynxArmAngles.Factory factory,
            double boomLength, double stickLength,
            double wristLength) {
        m_factory = factory;
        m_boomLength = boomLength;
        m_stickLength = stickLength;
        m_wristLength = wristLength;
        twodof = new TwoDofKinematics(boomLength, stickLength);
    }

    /**
     * TODO: this should be a Transform3d not a Translation3d.
     * 
     * @returns relative to the arm origin, which is the intersection of the
     *          swing and boom axes.
     *          Facing the keys, +x is right, +y is forward, +z is up.
     */
    public Translation3d forward(LynxArmAngles joints) {
        // first find the 2d solution, in the 2dof axes (x up, y fwd)
        // this is what 2dof calls "x"
        double up = m_boomLength * Math.cos(joints.boomRad())
                + m_stickLength * Math.cos(joints.stickRad() + joints.boomRad())
                + m_wristLength * Math.cos(joints.wristRad() + joints.stickRad() + joints.boomRad());
        // this is what 2dof calls "y"
        double out = boomOut(joints) + stickOut(joints) + wristOut(joints);
        // then rotate it by the swing.
        return new Translation3d(
                -1 * out * Math.sin(joints.swingRad()),
                out * Math.cos(joints.swingRad()),
                up);
    }

    double boomOut(LynxArmAngles joints) {
        return m_boomLength * Math.sin(joints.boomRad());
    }

    double stickOut(LynxArmAngles joints) {
        return m_stickLength * Math.sin(joints.stickRad() + joints.boomRad());
    }

    double wristOut(LynxArmAngles joints) {
        return m_wristLength * Math.sin(joints.wristRad() + joints.stickRad() + joints.boomRad());
    }

    /**
     * TODO: the input should be a Pose3d, not a Translation3d+angle.
     * 
     * since 3dof in 2d is underconstrained, specify
     * the wrist angle here to get a single solution
     * 
     * @param position              relative to the arm origin, which is the
     *                              intersection of the
     *                              swing and boom axes.
     * @param wristAngleAbsoluteRad relative to horizontal, positive up, which is
     *                              the reverse of the armangles direction.
     * @param twist                 passthrough [0,1]
     * @param grip                  passthrough [0,1]
     */
    public LynxArmAngles inverse(
            Translation3d position,
            double wristAngleAbsoluteRad,
            double twist,
            double grip) {

        double swing = -1.0 * safeAtan(position.getX(), position.getY());

        double up = position.getZ();
        double out = Math.hypot(position.getX(), position.getY());

        out = out - m_wristLength * Math.cos(wristAngleAbsoluteRad);
        up = up - m_wristLength * Math.sin(wristAngleAbsoluteRad);

        ArmAngles a = twodof.inverse(new Translation2d(up, out));
        if (a == null) {
            throw new IllegalArgumentException("up" + up + " out " + out);
        }
        double wristAngleRelative = Math.PI / 2 - wristAngleAbsoluteRad - a.th1 - a.th2;
        return m_factory.fromRad(swing, a.th1, a.th2, wristAngleRelative, twist, grip);
    }

    /**
     * If both x and y are zero, return zero.
     * 
     * @param x along the keyboard, right positive
     * @param y towards the keyboard
     */
    private double safeAtan(double x, double y) {
        if (Math.abs(x) < 1e-3 && Math.abs(y) < 1e-3) {
            return 0;
        }
        //// NOTE switching x and y here
        return Math.atan2(x, y);
    }

}