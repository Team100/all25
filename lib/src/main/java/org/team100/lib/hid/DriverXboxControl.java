package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.EnumLogger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a Microsoft Xbox controller, Logitech F310, or similar.
 * 
 * Controls mapping (please keep this in sync with the code below):
 * 
 * <pre>
 * left trigger [0,1]     == medium speed
 * left bumper button     == slow speed
 * left stick x [-1,1]    == omega
 * left stick y [-1,1]    ==
 * left stick button      == drive-to-amp
 * dpad/pov angle [0,360] == snaps
 * "back" button          == reset 0 rotation
 * "start" button         == reset 180 rotation
 * right stick x [-1,1]   == x velocity
 * right stick y [-1,1]   == y velocity
 * right stick button     == 
 * x button               == full cycle
 * y button               == drive to note
 * a button               == lock rotation to amp
 * b button               == aim and shoot
 * right trigger [0,1]    ==
 * right bumper button    ==
 * </pre>
 * 
 * Do not use stick buttons, they are prone to stray clicks
 */
public class DriverXboxControl implements DriverControl {
    private static final double kDeadband = 0.1;
    private static final double kExpo = 0.65;
    private static final double kMedium = 0.5;
    private static final double kSlow = 0.15;

    private final XboxController m_controller;
    private final DoubleLogger m_log_right_y;
    private final DoubleLogger m_log_right_x;
    private final DoubleLogger m_log_left_x;
    private final EnumLogger m_log_speed;

    public DriverXboxControl(LoggerFactory parent) {
        m_controller = new XboxController(0);
        LoggerFactory child = parent.child(this);
        m_log_right_y = child.doubleLogger(Level.TRACE, "Xbox/right y");
        m_log_right_x = child.doubleLogger(Level.TRACE, "Xbox/right x");
        m_log_left_x = child.doubleLogger(Level.TRACE, "Xbox/left x");
        m_log_speed = child.enumLogger(Level.TRACE, "control_speed");
    }

    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    @Override
    public double shooterPivot() {
        return -1.0 * m_controller.getLeftY();
    }

    /**
     * Applies expo to the magnitude of the cartesian input, since these are "round"
     * joysticks.
     */
    @Override
    public Velocity velocity() {
        final double rightY = m_controller.getRightY();
        final double rightX = m_controller.getRightX();
        final double leftX = m_controller.getLeftX();
        m_log_right_y.log(() -> rightY);
        m_log_right_x.log(() -> rightX);
        m_log_left_x.log(() -> leftX);

        double dx = 0;
        double dy = 0;
        double x = -1.0 * clamp(rightY, 1);
        double y = -1.0 * clamp(rightX, 1);
        double r = Math.hypot(x, y);
        if (r > kDeadband) {
            double expoR = expo(r, kExpo);
            double ratio = expoR / r;
            dx = ratio * x;
            dy = ratio * y;
        } else {
            dx = 0;
            dy = 0;
        }

        double dtheta = expo(deadband(-1.0 * clamp(leftX, 1), kDeadband, 1), kExpo);

        Speed speed = speed();
        m_log_speed.log(() -> speed);

        switch (speed) {
            case SLOW:
                return new Velocity(kSlow * dx, kSlow * dy, kSlow * dtheta);
            case MEDIUM:
                return new Velocity(kMedium * dx, kMedium * dy, kMedium * dtheta);
            default:
                return new Velocity(dx, dy, dtheta);
        }
    }

    public boolean shoot() {
        return m_controller.getRightBumperButton();
    }

    /**
     * This used to be public and affect everything; now it just affects the
     * velocity() output above.
     */
    private Speed speed() {
        if (m_controller.getLeftBumperButton())
            return Speed.SLOW;
        if (m_controller.getLeftTriggerAxis() > .9)
            return Speed.MEDIUM;
        return Speed.NORMAL;
    }

    @Override
    public Rotation2d desiredRotation() {
        double desiredAngleDegrees = m_controller.getPOV();

        if (desiredAngleDegrees < 0) {
            return null;
        }
        return Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
    }

    @Override
    public boolean resetRotation0() {
        return m_controller.getBackButton();
    }

    @Override
    public boolean resetRotation180() {
        return m_controller.getStartButton();
    }

    @Override
    public boolean fullCycle() {
        return m_controller.getXButton();
    }

    @Override
    public boolean driveToObject() {
        return m_controller.getYButton();
    }
    
    @Override
    public boolean driveToTag() {
        return m_controller.getAButton();
    }

    @Override
    public boolean testTrajectory() {
        return m_controller.getBButton();
    }

    @Override
    public boolean test() {
        return false;
    }

}
