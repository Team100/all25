package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Sets up the Lynxmotion arm with all axes. */
public class LynxArmSetup implements Runnable {
    private final LynxArm m_arm;
    private final LynxArmVisualizer m_viz;

    public LynxArmSetup(XboxController m_controller) {
        m_arm = new LynxArm();
        m_viz = new LynxArmVisualizer(m_arm::getPosition);

        MoveCommand move1 = m_arm.moveTo(new Pose3d(0.15, 0.1, 0.1, new Rotation3d(0, Math.PI / 2, 0)));
        new Trigger(m_controller::getAButton).whileTrue(move1);
        MoveCommand move2 = m_arm.moveTo(new Pose3d(0.15, 0.1, 0, new Rotation3d(0, Math.PI / 2, 0)));
        new Trigger(m_controller::getBButton).whileTrue(move2);
        MoveCommand move3 = m_arm.moveTo(new Pose3d(0.15, -0.1, 0.1, new Rotation3d(0, Math.PI / 2, 0)));
        new Trigger(m_controller::getXButton).whileTrue(move3);

        MoveCommand move4 = m_arm.moveTo(new Pose3d(0.2, -0.12, 0.05, new Rotation3d(0, Math.PI / 2, 0)));
        MoveCommand move5 = m_arm.moveTo(new Pose3d(0.2, -0.12, 0.0, new Rotation3d(0, Math.PI / 2, 0)));
        MoveCommand move6 = m_arm.moveTo(new Pose3d(0.2, -0.12, 0.05, new Rotation3d(0, Math.PI / 2, 0)));
        MoveCommand move7 = m_arm.moveTo(new Pose3d(0.2, 0.12, 0.05, new Rotation3d(0, Math.PI / 2, 0)));
        MoveCommand move8 = m_arm.moveTo(new Pose3d(0.2, 0.12, 0.0, new Rotation3d(0, Math.PI / 2, 0)));
        MoveCommand move9 = m_arm.moveTo(new Pose3d(0.2, 0.12, 0.05, new Rotation3d(0, Math.PI / 2, 0)));
        new Trigger(m_controller::getYButton).whileTrue(
                Commands.sequence(
                        move4.until(move4::done),
                        move5.until(move5::done),
                        move6.until(move6::done),
                        move7.until(move7::done),
                        move8.until(move8::done),
                        move9.until(move9::done)));

        m_arm.setDefaultCommand(m_arm.moveHome());
    }

    @Override
    public void run() {
        m_viz.periodic();
    }

}
