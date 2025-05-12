package org.team100.lib.motion.lynxmotion_arm;

import java.util.OptionalDouble;

import org.team100.lib.motion.lynxmotion_arm.TwoDofKinematics.TwoDofArmConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class LynxArmKinematics {
    /**
     * Each joint is relative to its parent link, in radians.
     * 
     * The grip is not included here, since it isn't really an aspect of kinematics
     * at all.
     * 
     * maybe these should be "rotations" instead of doubles?
     * 
     * swing is optional because it can be indeterminate, for end positions
     * *on* the swing axis.
     * 
     * @param swing
     * @param boom
     * @param stick
     * @param wrist
     * @param twist
     */
    public record LynxArmConfig(
            OptionalDouble swing,
            double boom,
            double stick,
            double wrist,
            OptionalDouble twist) {
        public Transform3d swingT() {
            if (swing.isEmpty()) {
                System.out.println("empty swing");
                return Transform3d.kZero;
            }
            return yaw(swing.getAsDouble());
        }

        public Transform3d boomT() {
            return pitch(boom);
        }

        public Transform3d stickT() {
            return pitch(stick);
        }

        public Transform3d wristT() {
            return pitch(wrist);
        }

        public Transform3d twistT() {
            if (twist.isEmpty()) {
                System.out.println("empty twist");
                return Transform3d.kZero;
            }
            return roll(twist.getAsDouble());
        }

        static Transform3d roll(double x) {
            return new Transform3d(0, 0, 0, new Rotation3d(x, 0, 0));
        }

        static Transform3d pitch(double x) {
            return new Transform3d(0, 0, 0, new Rotation3d(0, x, 0));
        }

        static Transform3d yaw(double x) {
            return new Transform3d(0, 0, 0, new Rotation3d(0, 0, x));
        }
    }

    public final double m_swingHeight;
    public final double m_boomLength;
    public final double m_stickLength;
    /**
     * Includes the link before the roll servo and also the length of the grip,
     * since they're always parallel.
     */
    public final double m_wristLength;

    private final TwoDofKinematics twodof;

    /** All distances are in meters. */
    public LynxArmKinematics(
            double swingHeight,
            double boomLength,
            double stickLength,
            double wristLength) {
        m_swingHeight = swingHeight;
        m_boomLength = boomLength;
        m_stickLength = stickLength;
        m_wristLength = wristLength;
        twodof = new TwoDofKinematics(boomLength, stickLength);
    }

    /** Translate along the x axis. */
    static Transform3d x(double x) {
        return new Transform3d(x, 0, 0, new Rotation3d());
    }

    /** Translate along the z axis. */
    static Transform3d z(double z) {
        return new Transform3d(0, 0, z, new Rotation3d());
    }

    /**
     * Applies the configuration transforms in order to obtain the end-effector
     * (grip center) pose relative to the arm origin, which is the on the tabletop
     * at the swing axis.
     */
    public Pose3d forward(LynxArmConfig joints) {
        return Pose3d.kZero
                .transformBy(joints.swingT())
                .transformBy(z(m_swingHeight))
                .transformBy(joints.boomT())
                .transformBy(x(m_boomLength))
                .transformBy(joints.stickT())
                .transformBy(x(m_stickLength))
                .transformBy(joints.wristT())
                .transformBy(x(m_wristLength))
                .transformBy(joints.twistT());

    }

    /**
     * Solve the inverse kinematics for the given end-effector pose.
     * 
     * Refer to the diagram
     * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
     */
    public LynxArmConfig inverse(final Pose3d end) {
        final OptionalDouble swing;
        final double boom;
        final double stick;
        final double wrist;
        final OptionalDouble twist;

        Translation2d translation = end.toPose2d().getTranslation();

        // To find the wrist joint location, follow the wrist backwards.
        Pose3d wristPose = end.transformBy(x(m_wristLength).inverse());

        // Solve the 2dof problem. The 2dof coordinates are cylindrical.
        Translation2d twodofEnd = new Translation2d(
                Math.hypot(wristPose.getX(), wristPose.getY()),
                wristPose.getZ() - m_swingHeight);
        TwoDofArmConfig twodofConfig = twodof.inverse(twodofEnd);

        boom = twodofConfig.q1();
        stick = twodofConfig.q2();

        Rotation3d endRotation = end.getRotation();
        if (translation.getNorm() < 1e-3) {
            // The pose is above the swing axis.
            if (endRotation.getAxis().isEqual(VecBuilder.fill(0, 0, 1), 0.01)) {
                // The twist angle is straight up.
                // Both twist and swing are indeterminate; the caller should decide what to do.
                swing = OptionalDouble.empty();
                twist = OptionalDouble.empty();
                wrist = Math.PI - boom - stick;
            } else if (endRotation.getAxis().isEqual(VecBuilder.fill(0, 0, -1), 0.01)) {
                // The twist angle is straight down.
                // Both twist and swing are indeterminate; the caller should decide what to do.
                swing = OptionalDouble.empty();
                twist = OptionalDouble.empty();
                wrist = -Math.PI - boom - stick;
            } else {
                // The end is on the swing axis, but the wrist is not, so the arm swing needs to
                // be
                // aligned with the end angle.
                double swingAngle = endRotation.getZ();

                swing = OptionalDouble.of(swingAngle);

                // to get the wrist rotation, project the swing-relative rotation
                Rotation3d swing3d = new Rotation3d(0, 0, swingAngle);
                Rotation3d swingRelative3d = endRotation.minus(swing3d);
                // to get the stick-relative wrist angle, subtract the joints
                wrist = swingRelative3d.getY() - boom - stick;
                // to get the twist, remove the wrist part; the remaining rotation around x is
                // twist.
                Rotation3d wrist3d = new Rotation3d(0, swingRelative3d.getY(), 0);
                Rotation3d twist3d = swingRelative3d.minus(wrist3d);
                twist = OptionalDouble.of(twist3d.getX());

            }
        } else {
            // Check that the swing angle matches the end angle.
            Rotation2d endRotation2d = endRotation.toRotation2d();
            Rotation2d swingAngle = translation.getAngle();
            if (Math.abs(endRotation2d.minus(swingAngle).getRadians()) > 1e-3) {
                throw new IllegalArgumentException("end rotation not parallel with wrist rotation");
            }
            swing = OptionalDouble.of(swingAngle.getRadians());

            // to get the wrist rotation, project the swing-relative rotation
            Rotation3d swing3d = new Rotation3d(swingAngle);
            Rotation3d swingRelative3d = endRotation.minus(swing3d);
            // to get the stick-relative wrist angle, subtract the joints
            wrist = swingRelative3d.getY() - boom - stick;
            // to get the twist, remove the wrist part; the remaining rotation around x is
            // twist.
            Rotation3d wrist3d = new Rotation3d(0, swingRelative3d.getY(), 0);
            Rotation3d twist3d = swingRelative3d.minus(wrist3d);
            twist = OptionalDouble.of(twist3d.getX());
        }

        return new LynxArmConfig(swing, boom, stick, wrist, twist);
    }

}