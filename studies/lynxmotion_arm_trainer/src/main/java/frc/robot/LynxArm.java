package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Lynxmotion AL5D trainer board.
 * 
 * The arm is a serial mechanism, and so it implements a series of
 * three-dimensional transforms: rotations (the joints) and translations (along
 * the links).
 * 
 * Since the AL5D is mostly planar, we're using the 2d convention, where each
 * link is a translation along its own x, and each joint rotation is also
 * aligned with the same parent link x.
 * 
 * This means that the "all joints at zero" state for the arm should be
 * stretched out along the world x axis.
 * 
 * overview:
 * 
 * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
 *
 * calibration:
 * 
 * https://docs.google.com/spreadsheets/d/1XCtQGnJABVWTuCkx0t6u_xJjezelqyBE2Rz7eqmoyh4
 * 
 * background:
 * 
 * https://motion.cs.illinois.edu/RoboticSystems/Kinematics.html
 * 
 * TODO: calibrate the PWM pulse width range.
 * TODO: move this to lib when it's done
 */
public class LynxArm extends SubsystemBase {
    private final CalibratedServo m_swing;
    private final CalibratedServo m_boom;
    private final CalibratedServo m_stick;
    private final CalibratedServo m_wrist;
    private final CalibratedServo m_twist;
    private final CalibratedServo m_grip;

    public LynxArm() {
        // all these implement the WPI normal coordinates:
        // x ahead, y left, z up.

        // yaw; joint zero is is in the middle of the servo range; unconstrained.
        m_swing = new CalibratedServo(0,
                new Clamp(-Math.PI, Math.PI),
                new AffineFunction(-3.216, 1.534));

        // these are placeholders
        // TODO: calibrate all the axes.

        // pitch; joint zero and servo zero are aligned; unconstrained.
        m_boom = new CalibratedServo(1,
                new Clamp(-Math.PI, Math.PI),
                new AffineFunction(-Math.PI, 0));

        // pitch; joint zero is servo max; constrained in the positive direction.
        m_stick = new CalibratedServo(2,
                new Clamp(0, 1.3),
                new AffineFunction(-Math.PI, Math.PI));

        // pitch; joint zero is in the middle of the servo range; unconstrained
        m_wrist = new CalibratedServo(3,
                new Clamp(-Math.PI, Math.PI),
                new AffineFunction(-Math.PI, Math.PI / 2));

        // roll; joint zero is in the middle of the servo range; unconstrained.
        m_twist = new CalibratedServo(4,
                new Clamp(-Math.PI, Math.PI),
                new AffineFunction(-Math.PI, Math.PI / 2));

        // the grip axis measures the width of the jaws.
        // TODO: calibrate in meters
        m_grip = new CalibratedServo(5,
                new Clamp(0, 0.02),
                new AffineFunction(-0.02, 0.02));
    }

}
