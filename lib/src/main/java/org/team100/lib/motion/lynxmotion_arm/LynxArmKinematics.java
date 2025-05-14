package org.team100.lib.motion.lynxmotion_arm;

import java.util.OptionalDouble;

import org.team100.lib.motion.lynxmotion_arm.TwoDofKinematics.TwoDofArmConfig;

import edu.wpi.first.math.MathUtil;
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
     */
    public record LynxArmConfig(
            OptionalDouble swing,
            double boom,
            double stick,
            double wrist,
            OptionalDouble twist) {
        public LynxArmConfig(double swing, double boom, double stick, double wrist, double twist) {
            this(OptionalDouble.of(swing), boom, stick, wrist, OptionalDouble.of(twist));
        }

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

    /**
     * Workspace pose of each joint. The rotation is the rotation of the parent
     * link, in workspace coordinates, not relative to the previous.
     */
    public record LynxArmPose(
            Pose3d p1,
            Pose3d p2,
            Pose3d p3,
            Pose3d p4,
            Pose3d p5) {
    }

    public final double m_swingHeight;
    public final double m_boomLength;
    public final double m_stickLength;
    /**
     * Includes the link before the twist servo and also the length of the grip,
     * since they're collinear.
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
    public LynxArmPose forward(LynxArmConfig joints) {
        Pose3d root = Pose3d.kZero;
        // the end of the swing axis, at the boom joint
        // TODO: this is a little bit wierd, translating in z. Maybe use x and a
        // rotation?
        Pose3d boomRoot = root.transformBy(joints.swingT()).transformBy(z(m_swingHeight));
        // the end of the boom, at the stick joint
        Pose3d stickRoot = boomRoot.transformBy(joints.boomT()).transformBy(x(m_boomLength));
        // the end of the stick, at the wrist joint
        Pose3d wristRoot = stickRoot.transformBy(joints.stickT()).transformBy(x(m_stickLength));
        // this is actually at the grip center but without the twist,
        // since the twist axis and the grip axis are collinear.
        Pose3d twistRoot = wristRoot.transformBy(joints.wristT()).transformBy(x(m_wristLength));
        // this is Tool Center Point.
        Pose3d grip = twistRoot.transformBy(joints.twistT());
        return new LynxArmPose(boomRoot, stickRoot, wristRoot, twistRoot, grip);
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

        // Solve the 2dof problem. The 2dof coordinates are different:
        // the x dimension is the radius in 3d xy.
        // the swing is limited to +/- pi/2, so if the 3d "x" value is negative (i.e.
        // quadrant 3 or 4) then the 2d x value should be too.
        double hypot = Math.hypot(wristPose.getX(), wristPose.getY()) * Math.signum(wristPose.getX());
        // 3d z becomes y, offset by the swing height.
        double twoDofY = wristPose.getZ() - m_swingHeight;
        Translation2d twoDofEnd = new Translation2d(
                hypot,
                twoDofY);
        TwoDofArmConfig twoDofConfig = twodof.inverse(twoDofEnd);

        // the 2d coordinates are inverted for convenience, so fix it here.
        boom = -1.0 * twoDofConfig.q1();
        stick = -1.0 * twoDofConfig.q2();

        Rotation3d endRotation = end.getRotation();
        if (translation.getNorm() < 1e-3) {
            System.out.println("The end pose is above the swing axis.");

            if (MathUtil.isNear(-Math.PI / 2, endRotation.getY(), 0.01)) {
                // The wrist axis is pointing up.
                // Both twist and swing are indeterminate; the caller should decide what to do.
                swing = OptionalDouble.empty();
                twist = OptionalDouble.empty();
                wrist = -Math.PI / 2 - boom - stick;
            } else if (MathUtil.isNear(Math.PI / 2, endRotation.getY(), 0.01)) {
                // The wrist axis is pointing down.
                // Both twist and swing are indeterminate; the caller should decide what to do.
                swing = OptionalDouble.empty();
                twist = OptionalDouble.empty();
                wrist = Math.PI / 2 - boom - stick;
            } else {
                // The end is on the swing axis, but the wrist is not, so the arm swing needs to
                // be aligned with the end angle.
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
            // the end pose is not on the swing axis, so we can use the pose translation to
            // get the swing angle.
            Rotation2d swingAngle = translation.getAngle();
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

            if (MathUtil.isNear(-Math.PI / 2, endRotation.getY(), 0.01)) {
                System.out.println("The wrist axis is straight up.");
            } else if (MathUtil.isNear(Math.PI / 2, endRotation.getY(), 0.01)) {
                System.out.println("The wrist axis is straight down");
            } else {
                // Check that the swing angle matches the end angle.
                Rotation2d endRotation2d = endRotation.toRotation2d();
                if (Math.abs(endRotation2d.minus(swingAngle).getRadians()) > 1e-3) {
                    // throw new IllegalArgumentException(
                    // String.format("end rotation %s not equal to swing %s",
                    // endRotation2d, swingAngle));
                    System.out.printf("end rotation %s not equal to swing %s for goal %s\n",
                            endRotation2d, swingAngle, poseStr(end));
                }
            }
        }

        return new LynxArmConfig(swing, boom, stick, wrist, twist);
    }

    String poseStr(Pose3d p) {
        return String.format("%f %f %f %f %f %f",
                p.getX(), p.getY(), p.getZ(),
                p.getRotation().getX(), p.getRotation().getY(), p.getRotation().getZ());
    }

}