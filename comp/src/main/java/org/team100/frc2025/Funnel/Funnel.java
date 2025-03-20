package org.team100.frc2025.Funnel;

import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550Factory;
import org.team100.lib.motor.SimulatedBareMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase implements Glassy {

    LinearMechanism m_starboardMech;
    LinearMechanism m_portMech;

    public Funnel(
            LoggerFactory parent,
            int starboardID,
            int portID) {
        LoggerFactory child = parent.child(this);

        int funnelSupplyLimit = 20;

        switch (Identity.instance) {
            case COMP_BOT -> {

                m_starboardMech = Neo550Factory.getNEO550LinearMechanism(getName(), child, funnelSupplyLimit,
                        starboardID, 1, MotorPhase.REVERSE, 1);
                m_portMech = Neo550Factory.getNEO550LinearMechanism(getName(), child, funnelSupplyLimit, portID, 1,
                        MotorPhase.FORWARD, 1);

                break;
            }
            default -> {
                SimulatedBareMotor starboardMotor = new SimulatedBareMotor(child, 100);
                SimulatedBareEncoder starboardEncoder = new SimulatedBareEncoder(child, starboardMotor);
                SimulatedBareMotor portMotor = new SimulatedBareMotor(child, 100);
                SimulatedBareEncoder portEncoder = new SimulatedBareEncoder(child, portMotor);
                m_starboardMech = new SimpleLinearMechanism(starboardMotor, starboardEncoder, 1, 1);
                m_portMech = new SimpleLinearMechanism(portMotor, portEncoder, 1, 1);
            }
        }
    }

    public void setStarboard(double value) {
        m_starboardMech.setDutyCycle(value);
    }

    public void setPort(double value) {
        m_portMech.setDutyCycle(value);
    }

    public void setFunnel(double value) {
        setStarboard(value);
        setPort(value);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
