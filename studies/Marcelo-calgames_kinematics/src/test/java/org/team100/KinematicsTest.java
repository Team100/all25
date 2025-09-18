package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class KinematicsTest {
    @Test
    void testForward() {
        Kinematics k = new Kinematics(0.3, 0.1,0);
        Config c = new Config(0, 0, 0);
        Pose2d p = k.forward(c);
        assertEquals(0.5, p.getX(), 0.001);
        assertEquals(0.0, p.getY(), 0.001);
        assertEquals(0, p.getRotation().getRadians(), 0.001);
    }

    @Test
    void testInverse() {
        Kinematics k = new Kinematics(0.3, 0.1,0);
        Pose2d p = new Pose2d(0.3, 0.1, new Rotation2d());
        Config c = k.inverse(p);
        assertEquals(0.0, c.pivotHeightM(), 0.001);
        assertEquals(0.0, c.pivotAngleRad(), 0.001);
        assertEquals(0.0, c.wristAngleRad(), 0.001);
        System.out.println("Pivot angle: " + c.pivotAngleRad());
        System.out.println("Wrist angle: " + c.wristAngleRad());
        System.out.println("Pivot height: " + c.pivotHeightM());
    }

}
