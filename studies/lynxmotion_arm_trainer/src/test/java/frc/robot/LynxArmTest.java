package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.lynxmotion_arm.LynxArmKinematics.LynxArmConfig;
import org.team100.lib.motion.lynxmotion_arm.LynxArmKinematics.LynxArmPose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class LynxArmTest {

    @Test
    void testTwist() {
        try (LynxArm m_arm = new LynxArm()) {
            Pose3d start = new Pose3d(0.15, -0.1, 0.1, new Rotation3d(0, Math.PI / 2, 0));
            m_arm.setPosition(start);
            Pose3d end = new Pose3d(0.15, 0.1, 0.1, new Rotation3d(0, Math.PI / 2, 0));
            System.out.printf("start %s\n", poseStr(start));
            System.out.printf("end %s\n", poseStr(end));
            for (double s = 0; s <= 1; s += 0.2) {
                Pose3d lerp = start.interpolate(end, s);
                m_arm.setPosition(lerp);
                // wrist should be pointing down the whole time
                LynxArmPose p = m_arm.getPosition();
                System.out.printf("s %f p1 %s\n", s, poseStr(p.p1()));
                System.out.printf("s %f p2 %s\n", s, poseStr(p.p2()));
                System.out.printf("s %f p3 %s\n", s, poseStr(p.p3()));
                System.out.printf("s %f p4 %s\n", s, poseStr(p.p4()));
                System.out.printf("s %f p5 %s\n", s, poseStr(p.p5()));
            }
        }
    }

    @Test
    void testRoundTrip() {
        try (LynxArm m_arm = new LynxArm()) {
            Pose3d setpoint = new Pose3d(
                    new Translation3d(0.15, 0.1, 0),
                    new Rotation3d(0, Math.PI / 2, 0));
            m_arm.setPosition(setpoint);
            // System.out.printf("setpoint %s\n", poseStr(setpoint));
            LynxArmConfig measuredConfig = m_arm.getMeasuredConfig();
            System.out.printf("measured  config %s\n", measuredConfig);
            LynxArmConfig commandedConfig = m_arm.getInverse(setpoint);
            System.out.printf("commanded config %s\n", commandedConfig);
            assertEquals(measuredConfig.stick(), commandedConfig.stick(), 0.001);
        }
    }

    @Test
    void testRoundTrip2() {
        try (LynxArm m_arm = new LynxArm()) {
            Pose3d setpoint = new Pose3d(
                    new Translation3d(0.165326, 0.056973, 0.085628),
                    new Rotation3d(0.172187, 0.923138, -0.154456));
            m_arm.setPosition(setpoint);
            System.out.printf("setpoint %s\n", poseStr(setpoint));
            LynxArmConfig measuredConfig = m_arm.getMeasuredConfig();
            System.out.printf("measured  config %s\n", measuredConfig);
            LynxArmConfig commandedConfig = m_arm.getInverse(setpoint);
            System.out.printf("commanded config %s\n", commandedConfig);
            assertEquals(measuredConfig.stick(), commandedConfig.stick(), 0.001);
        }
    }

    String poseStr(Pose3d p) {
        return String.format("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f",
                p.getX(), p.getY(), p.getZ(),
                p.getRotation().getX(), p.getRotation().getY(), p.getRotation().getZ());
    }
}
