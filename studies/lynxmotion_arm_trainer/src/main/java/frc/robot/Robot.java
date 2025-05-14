package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;
    private final DemoLED m_led;
    private final XboxController m_controller;
    private final Runnable m_setup;

    public Robot() {
        m_robotContainer = new RobotContainer();
        m_led = new DemoLED();
        m_controller = new XboxController(0);
        // m_led.setDefaultCommand(m_led.sweep());

        /////////////////////////////
        /// 
        /// Choose one of the setups
        /// 

        // calibrate one axis at a time
        // m_setup = new CalibratorSetup(m_controller, m_led);

        // two-dof arm
        // m_setup = new LynxArmTwoDofSetup(m_controller);

        // five-dof arm
        m_setup = new LynxArmSetup(m_controller);

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_setup.run();
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
