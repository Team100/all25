package org.team100.studies.state_based_lynxmotion_arm.state;

import org.team100.studies.state_based_lynxmotion_arm.subsystems.Arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Describes actions and the graph of transitions. */
public class Chart {
    private final XboxController m_controller;
    private final Arm m_arm;
    private State m_current;

    public Chart(XboxController controller, Arm arm) {
        m_controller = controller;
        m_arm = arm;
        m_current = State.HOME;
        // print on state entry
        in(State.HOME).onTrue(Commands.print("HOME"));
        in(State.AWAY).onTrue(Commands.print("AWAY"));
        // transition on button press
        in(State.HOME).and(m_controller::getAButton).onTrue(set(State.AWAY));
        in(State.AWAY).and(m_controller::getBButton).onTrue(set(State.HOME));
        // execute the action for each state
        in(State.HOME).onTrue(arm.goHome());
        in(State.AWAY).onTrue(arm.goAway());
        // time out the away state
        in(State.AWAY).and(arm::awayTimerExpired).onTrue(set(State.HOME));
    }

    private Trigger in(State state) {
        return new Trigger(() -> m_current == state);
    }

    private Command set(State state) {
        return Commands.runOnce(() -> {
            m_current = state;
        });
    }

}
