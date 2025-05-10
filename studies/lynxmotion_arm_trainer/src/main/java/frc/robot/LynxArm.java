package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Lynxmotion AL5D board.
 * 
 * Results from calibration:
 * 
 * 0.0:90
 * 0.1:71
 * 0.2:50
 * 0.3:31
 * 0.4:14
 * 0.5:-6
 * 0.6:-24
 * 0.7:-41
 * 0.8:-61
 * 0.9:-77
 * 1.0:-94
 * 
 * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
 * 
 * TODO: move this to lib when it's done
 */
public class LynxArm extends SubsystemBase {
    private final CalibratedServo m_swing;
    // private final Servo m_boom;
    // private final Servo m_stick;
    // private final Servo m_wrist;
    // private final Servo m_twist;
    // private final Servo m_grip;

    public LynxArm() {
        m_swing = new CalibratedServo(0);
        // m_boom = new Servo(1);
        // m_stick = new Servo(2);
        // m_wrist = new Servo(3);
        // m_twist = new Servo(4);
        // m_grip = new Servo(5);
    }

}
