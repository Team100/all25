package org.team100.frc2025.Wrist;

import java.util.function.BooleanSupplier;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550CANSparkMotor;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralTunnel extends SubsystemBase implements Glassy {
    // private final LinearMechanism m_coralMech;
    private LaserCan laserCAN;

    private final Neo550CANSparkMotor m_motor;

    private final BooleanLogger m_logLeftLimitSwitch;
    private final BooleanLogger m_logRightLimitSwitch;

    private final BooleanSupplier m_leftLimitSwitch;
    private final BooleanSupplier m_rightLimitSwitch;

    public CoralTunnel(LoggerFactory parent, int algaeID, int coralID) {

        LoggerFactory child = parent.child(this);
        int coralCurrentLimit = 20;

        m_logRightLimitSwitch = child.booleanLogger(Level.TRACE, "right");
        m_logLeftLimitSwitch = child.booleanLogger(Level.TRACE, "left");

        switch (Identity.instance) {
            case COMP_BOT -> {
                // m_coralMech = Neo550Factory.getNEO550LinearMechanism(getName(), child,
                // coralCurrentLimit, coralID, 1,
                // MotorPhase.FORWARD, 1);
                // m_motor = new SparkMax(25, MotorType.kBrushless);
                m_motor = new Neo550CANSparkMotor(
                        child,
                        25,
                        MotorPhase.REVERSE,
                        coralCurrentLimit,
                        Feedforward100.makeNeo550(),
                        new PIDConstants());

                m_leftLimitSwitch = () -> m_motor.getReverseLimitSwitch();
                m_rightLimitSwitch = () -> m_motor.getForwardLimitSwitch();

            }
            default -> {
                // m_motor = new SimulatedBareMotor(child, 100);
                m_leftLimitSwitch = () -> false;
                m_rightLimitSwitch = () -> false;
                m_motor = null;
            }
        }

    }

    @Override
    public void periodic() {
        m_logRightLimitSwitch.log(() -> m_rightLimitSwitch.getAsBoolean());
        m_logLeftLimitSwitch.log(() -> m_leftLimitSwitch.getAsBoolean());
    }

    public boolean hasAlgae() {
        return m_leftLimitSwitch.getAsBoolean() || m_rightLimitSwitch.getAsBoolean();
    }

    public void setCoralMotor(double value) {
        if (m_motor == null)
            return;
        m_motor.setDutyCycle(value);
    }
}
