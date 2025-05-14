package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LynxArmTwoDofSetup implements Runnable {
    private final LynxArmTwoDof m_arm;
    private final LynxArmVisualizer m_viz;

    public LynxArmTwoDofSetup(XboxController m_controller) {
        m_arm = new LynxArmTwoDof();
        m_viz = new LynxArmVisualizer(m_arm::getPose);

        MoveCommandTwoDof move1 = m_arm.moveTo(new Translation2d(0.2, 0.1));
        new Trigger(m_controller::getAButton).whileTrue(move1);
        MoveCommandTwoDof move2 = m_arm.moveTo(new Translation2d(0.2, 0));
        new Trigger(m_controller::getBButton).whileTrue(move2);
        MoveCommandTwoDof move3 = m_arm.moveTo(new Translation2d(0.3, 0.1));
        new Trigger(m_controller::getXButton).whileTrue(move3);

        MoveCommandTwoDof move4 = m_arm.moveTo(new Translation2d(0.2, 0.05));
        MoveCommandTwoDof move5 = m_arm.moveTo(new Translation2d(0.2, 0.0));
        MoveCommandTwoDof move6 = m_arm.moveTo(new Translation2d(0.2, 0.05));
        MoveCommandTwoDof move7 = m_arm.moveTo(new Translation2d(0.4, 0.05));
        MoveCommandTwoDof move8 = m_arm.moveTo(new Translation2d(0.4, 0.0));
        MoveCommandTwoDof move9 = m_arm.moveTo(new Translation2d(0.4, 0.05));
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
