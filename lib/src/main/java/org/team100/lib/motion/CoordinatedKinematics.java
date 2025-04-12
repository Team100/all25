package org.team100.lib.motion;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Kinematics for the arm/elevator combination.
 * 
 * This system is represented by two measurements: height of the pivot, and
 * angle of the arm. The angle is measured from vertical. There are physical
 * limits in the real system; these are not represented here.
 * 
 * The cartesian coordinate orientation is with X vertical.
 */
public class CoordinatedKinematics {
    public static record Joints(double height, double angle) {
    }

    private final double l;

    public CoordinatedKinematics(double l) {
        this.l = l;
    }

    /**
     * The position of the end-effector based on the two measurements.
     * 
     * The forward kinematics are unique.
     */
    public Translation2d forward(Joints joints) {
        double x = joints.height + l * Math.cos(joints.angle);
        double y = l * Math.sin(joints.angle);
        return new Translation2d(x, y);
    }

    /**
     * The measurements required to achieve the given coordinates.
     * 
     * The inverse kinematics are not unique: there are two ways to get to almost
     * all points in the envelope: the "arm pointing up" orientation and the "arm
     * pointing down" orientation. The use of arcsin below prefers the "arm pointing
     * up" case.
     * 
     * There are also unreachable points outside the envelope; in that case we
     * return null.
     */
    public Joints inverse(Translation2d t) {
        double angle = Math.asin(t.getY() / l);
        double height = t.getX() - l * Math.cos(angle);
        if (Double.isNaN(angle) || Double.isNaN(height))
            return null;
        return new Joints(height, angle);

    }

}
