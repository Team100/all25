package org.team100.studies.state_based_lynxmotion_arm.state;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.function.BooleanSupplier;

import org.team100.studies.state_based_lynxmotion_arm.subsystems.Arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This is the same logic as Chart but using the behavior tree style that WPI
 * Commands support.
 * 
 * This seems better. The little factory methods are required so that each
 * collection gets its own instance of the reused parts.
 */
public class Alternative {
    private final Arm m_arm;
    private final EventLoop m_loop;

    public Alternative(XboxController controller, Arm arm) {
        m_arm = arm;
        m_loop = new EventLoop();
        on(controller::getAButton, sequence(away(), pause(), home()));
        on(controller::getBButton, home());
        on(controller::getXButton, sequence(east(), north(), west(), home()));
        on(controller::getYButton, sequence(west(), home()));
    }

    public void poll() {
        m_loop.poll();
    }

    private Command east() {
        return m_arm.goEast().until(m_arm::isEast);
    }

    private Command north() {
        return m_arm.goNorth().until(m_arm::isNorth);
    }

    private Command west() {
        return m_arm.goWest().until(m_arm::isWest);
    }

    private Command pause() {
        return waitSeconds(1);
    }

    private Command away() {
        return m_arm.goAway().until(m_arm::isAway);
    }

    private Command home() {
        return m_arm.goHome().until(m_arm::isHome);
    }

    private Trigger on(BooleanSupplier condition, Command command) {
        // Using whileTrue here means you can interrupt the command by
        // letting go of the button, which seems like the right thing.
        //
        // Note: if you release and then press the same button again, the sequence will
        // start again, which may not be what you want.
        // 
        // TODO: think about how to pick up where you left off?
        return new Trigger(m_loop, condition).whileTrue(command);
    }
}
