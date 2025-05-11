package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;
    private final DemoLED m_led;
    private final XboxController m_controller;
    private final AxisCalibrator m_calibrator;

    public Robot() {
        m_robotContainer = new RobotContainer();
        m_led = new DemoLED();
        m_controller = new XboxController(0);
        m_calibrator = new AxisCalibrator(0);
        new Trigger(m_controller::getXButton).onTrue(m_calibrator.step());
        // m_led.setDefaultCommand(m_led.sweep());
        m_led.setDefaultCommand(m_led.indicateCalibration(m_calibrator));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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

}
