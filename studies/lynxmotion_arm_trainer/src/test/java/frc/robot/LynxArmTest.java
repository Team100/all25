package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.lynxmotion_arm.LynxArmKinematics.LynxArmConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class LynxArmTest {
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
        return String.format("%f %f %f %f %f %f",
                p.getX(), p.getY(), p.getZ(),
                p.getRotation().getX(), p.getRotation().getY(), p.getRotation().getZ());
    }
}
