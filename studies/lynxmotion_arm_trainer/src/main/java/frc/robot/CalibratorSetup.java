package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Sets up the axis calibrator to use the Lynxmotion arm. */
public class CalibratorSetup implements Runnable {
    private final AxisCalibrator m_calibrator;

    public CalibratorSetup(XboxController m_controller, DemoLED m_led) {
        m_calibrator = new AxisCalibrator(2);
        new Trigger(m_controller::getXButton).onTrue(m_calibrator.step());
        m_led.setDefaultCommand(m_led.indicateCalibration(m_calibrator));
    }

    @Override
    public void run() {
        //
    }

}
