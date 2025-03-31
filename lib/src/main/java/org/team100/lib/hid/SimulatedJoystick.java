package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Simulated Joystick.
 * 
 * This is used for simulation.
 */
public class SimulatedJoystick implements DriverControl {
    private static final boolean DEBUG = false;
    private static final double kDeadband = 0.02;
    private static final double kExpo = 0.5;

    private final GenericHID m_hid;

    protected SimulatedJoystick() {
        m_hid = new GenericHID(0);
    }

    @Override
    public String getHIDName() {
        return m_hid.getName();
    }

    @Override
    public boolean trigger() {
        // return button(1);
        return false;
    }

    @Override
    public boolean fullCycle() {
        // return button(1);
        return false;
    }

    @Override
    public boolean useReefLock() {
        return button(1);
    }

    @Override
    public boolean testTrajectory() {
        return button(2);
    }

    @Override
    public boolean test() {
        return button(3);
    }

    @Override
    public boolean driveToObject() {
        return button(4);
    }

    @Override
    public boolean button5() {
        // return button(5);
        return false;
    }

    @Override
    public boolean driveToTag() {
        return button(6);
    }

    @Override
    public boolean resetRotation0() {
        return button(7); // "F1"
    }

    @Override
    public boolean resetRotation180() {
        return button(8); // "F2"
    }

    /**
     * Applies expo to each axis individually, works for "square" joysticks.
     * The square response of this joystick should be clamped by the consumer.
     */
    @Override
    public DriverControl.Velocity velocity() {
        double dx = expo(deadband(-1.0 * clamp(axis(1), 1), kDeadband, 1), kExpo);
        double dy = expo(deadband(-1.0 * clamp(axis(0), 1), kDeadband, 1), kExpo);
        double dtheta = expo(deadband(clamp(axis(2), 1), kDeadband, 1), kExpo);
        DriverControl.Velocity velocity = new DriverControl.Velocity(dx, dy, dtheta);
        if (DEBUG)
            Util.printf("SimulatedJoystick %s\n", velocity);
        return velocity;
    }

    @Override
    public Rotation2d desiredRotation() {
        // POV 2 is the center one
        double desiredAngleDegrees = m_hid.getPOV(2);
        if (desiredAngleDegrees < 0) {
            return null;
        }
        return Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
    }

    /**
     * For now, this knows the field-relative target.
     */
    @Override
    public Translation2d target() {
        if (m_hid.getRawButton(3)) {
            // alternate target is closer to the left side
            return new Translation2d(6, 4);
        } else {
            // default target is kinda mid-field
            return new Translation2d(0.431985, 5.446929);
        }

        // return new Translation2d(0.431985, 5.446929);
    }

    //////////////////

    private double axis(int axis) {
        return m_hid.getRawAxis(axis);
    }

    private boolean button(int button) {
        return m_hid.getRawButton(button);
    }
}
