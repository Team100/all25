package org.team100.lib.examples.motion;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Sometimes you don't need fancy positional profiles and feedback controls, you
 * just need something to spin on demand.
 * 
 * A common example is intake rollers: they don't need velocity control, they
 * just need to spin.
 * 
 * This example illustrates this use-case: the simplest possible open-loop
 * control.
 * 
 * This class extends SubsystemBase, to be compatible with the scheduler, and it
 * implements Glassy, so that the logger will use the class name.
 */
public class OpenLoopSubsystem extends SubsystemBase implements Glassy {
    private final BareMotor m_motor;

    public OpenLoopSubsystem(LoggerFactory parent) {
        LoggerFactory log = parent.child(this);
        /*
         * Here we use the Team 100 "Identity" mechanism to allow different
         * configurations for different hardware. The most important distinction here is
         * for simulation.
         */
        switch (Identity.instance) {
            case COMP_BOT -> {
                int canId = 1;
                int supplyLimit = 60;
                int statorLimit = 90;
                PIDConstants PID = PIDConstants.makeVelocityPID(0.3);
                // you should make a case in the feedforward class for your constants
                Feedforward100 FF = Feedforward100.makeSimple();
                m_motor = new Falcon6Motor(
                        log, canId, MotorPhase.FORWARD, supplyLimit, statorLimit, PID, FF);
            }
            default -> {
                m_motor = new SimulatedBareMotor(log, 600);
            }
        }
    }

    ///////////////////////////////////////////////////////
    //
    // ACTIONS
    //
    // These methods make the subsystem do something.

    public void setDutyCycle(double dutyCycle) {
        m_motor.setDutyCycle(dutyCycle);
    }

    public void setVelocity(double velocity) {
        m_motor.setVelocity(velocity, 0, 0);
    }

    ///////////////////////////////////////////////////////
    //
    // COMMANDS
    //
    // For single-subsystem actions, these actuator commands are the cleanest way to
    // do it. Multi-subsystem actions would need to use the methods above.
    //

    public Command forward() {
        return run(() -> {
            setDutyCycle(1.0);
        });
    }

    public Command reverse() {
        return run(() -> {
            setDutyCycle(1.0);
        });
    }

    @Override
    public void periodic() {
        m_motor.periodic();
    }
}
