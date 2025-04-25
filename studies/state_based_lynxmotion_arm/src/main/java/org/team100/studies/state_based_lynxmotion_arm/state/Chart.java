package org.team100.studies.state_based_lynxmotion_arm.state;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

import org.team100.lib.framework.EventLoop100;
import org.team100.lib.framework.Trigger100;
import org.team100.studies.state_based_lynxmotion_arm.subsystems.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** Describes actions and the graph of transitions. */
public class Chart {
    private final XboxController m_controller;
    private final Arm m_arm;
    private final EventLoop100 m_loop;
    private final Set<State> m_buffer;
    // TODO: buffer updates: triggers fill a queue, then the queue is applied all at
    // once. if the queue contains more than one thing, explode.
    private State m_current;

    public Chart(XboxController controller, Arm arm) {
        m_controller = controller;
        m_arm = arm;
        m_loop = new EventLoop100();
        m_buffer = new LinkedHashSet<>();
        m_current = State.HOME;
        // print on state entry
        in(State.HOME).onTrue(print("HOME").withName("enter home"));
        in(State.AWAY).onTrue(print("AWAY").withName("enter away"));
        in(State.EAST).onTrue(print("EAST"));
        in(State.WEST).onTrue(print("WEST"));
        in(State.WEST).onTrue(print("NORT"));
        // guarded transition on button press
        in(State.HOME).and(m_controller::getAButton).onTrue(set(State.AWAY).withName("a: home->away"));
        in(State.AWAY).and(m_controller::getBButton).onTrue(set(State.HOME));
        // transitions from any state
        always(m_controller::getXButton).onTrue(set(State.EAST));
        always(m_controller::getYButton).onTrue(set(State.WEST));
        // this would produce a conflict: same condition, different transition
        // TODO: write a test about this.
        // always(m_controller::getYButton).onTrue(set(State.EAST));
        // a simple sequence
        in(State.EAST).and(arm::isEast).onTrue(set(State.NORTH));
        in(State.NORTH).and(arm::isNorth).onTrue(set(State.WEST));
        in(State.WEST).and(arm::isWest).onTrue(set(State.HOME));
        // execute the action for each state
        in(State.HOME).onTrue(arm.goHome());
        in(State.AWAY).onTrue(arm.goAway());
        in(State.EAST).onTrue(arm.goEast());
        in(State.WEST).onTrue(arm.goWest());
        in(State.NORTH).onTrue(arm.goNorth());
        // once the arm reaches the away state, start a timer and send it
        // back home after the timer expires. The timer used to be in the
        // arm but it seems like it should be here instead; the arm doesn't
        // care about time.
        Timer timer = new Timer();
        in(State.AWAY).and(arm::isAway).onTrue(
                runOnce(timer::restart).withName("start timer"));
        in(State.AWAY).and(arm::isAway).and(() -> timer.hasElapsed(1)).onTrue(
                runOnce(timer::stop).withName("stop timer")
                        .andThen(timer::reset).withName("reset timer")
                        .andThen(set(State.HOME).withName("timeout: away->home")));
    }

    /** Evaluate all the triggers, some of which may queue a transition. */
    public void poll() {
        m_loop.pollEvents();
        commit();
    }

    /** Commit the transition. */
    private void commit() {
        if (m_buffer.isEmpty())
            return;
        if (m_buffer.size() > 1) {
            throw new IllegalStateException("conflicting state updates");
        }
        m_current = m_buffer.iterator().next();
        // System.out.println("new committed state: " + m_current);
        m_buffer.clear();
    }

    private Trigger100 in(State state) {
        return new Trigger100(m_loop, () -> m_current == state);
    }

    private Trigger100 always(BooleanSupplier condition) {
        return new Trigger100(m_loop, condition);
    }

    private Command set(State state) {
        return runOnce(() -> {
            m_buffer.add(state);
            // m_current = state;
        });
    }

}
