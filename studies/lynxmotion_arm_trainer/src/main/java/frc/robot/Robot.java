package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;
    private final DemoLED m_led;
    private final XboxController m_controller;
    // private final AxisCalibrator m_calibrator;
    private final LynxArmVisualizer m_viz;
    private final LynxArm m_arm;

    public Robot() {
        m_robotContainer = new RobotContainer();
        m_led = new DemoLED();
        m_controller = new XboxController(0);
        // m_led.setDefaultCommand(m_led.sweep());

        // calibrator
        //
        // m_calibrator = new AxisCalibrator(2);
        // new Trigger(m_controller::getXButton).onTrue(m_calibrator.step());
        // m_led.setDefaultCommand(m_led.indicateCalibration(m_calibrator));

        // arm
        m_arm = new LynxArm();
        m_viz = new LynxArmVisualizer(m_arm);

        new Trigger(m_controller::getAButton).whileTrue(
                m_arm.moveTo(new Pose3d(0.15, 0.1, 0.1, new Rotation3d(0, Math.PI / 2, 0))));
        new Trigger(m_controller::getBButton).whileTrue(
                m_arm.moveTo(new Pose3d(0.15, 0.1, 0, new Rotation3d(0, Math.PI / 2, 0))));
        new Trigger(m_controller::getXButton).whileTrue(
                m_arm.moveTo(new Pose3d(0.15, -0.1, 0.1, new Rotation3d(0, Math.PI / 2, 0))));
        // new Trigger(m_controller::getYButton).whileTrue(
        // m_arm.moveTo(new Pose3d(0.15, -0.1, 0, new Rotation3d(0, Math.PI / 2, 0))));

        // move only the twist axis here
        new Trigger(m_controller::getYButton).whileTrue(
                m_arm.moveTo(new Pose3d(0.2, 0, 0.1, new Rotation3d(Math.PI/2, 0, 0))));

        m_arm.setDefaultCommand(m_arm.moveHome());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_viz.periodic();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void simulationInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

}
