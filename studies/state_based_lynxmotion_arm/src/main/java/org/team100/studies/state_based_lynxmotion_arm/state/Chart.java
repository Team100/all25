package org.team100.studies.state_based_lynxmotion_arm.state;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Describes actions and the graph of transitions. */
public class Chart {
    private final XboxController m_controller;
    private State m_current;

    public Chart(XboxController controller) {
        m_controller = controller;
        m_current = State.HOME;
        in(State.HOME).onTrue(Commands.print("HOME"));
        in(State.AWAY).onTrue(Commands.print("AWAY"));
        in(State.HOME).and(m_controller::getAButton).onTrue(set(State.AWAY));
        in(State.AWAY).and(m_controller::getBButton).onTrue(set(State.HOME));
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
