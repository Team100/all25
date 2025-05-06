package org.team100.studies.state_based_lynxmotion_arm.state;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Some test examples */
public class TestChart {
    private final XboxController m_controller;

    private boolean foo;

    public TestChart(XboxController controller) {
        m_controller = controller;
    }

    void good() {
        // transition foo -> not-foo
        new Trigger(() -> foo).and(m_controller::getXButton).onTrue(runOnce(() -> {
            foo = false;
        }));
        // enter foo
        new Trigger(() -> foo).onTrue(print("A"));
        // exit foo
        new Trigger(() -> foo).onFalse(print("B"));
        // transition not-foo -> foo
        new Trigger(() -> !foo).and(m_controller::getYButton).onTrue(runOnce(() -> {
            foo = true;
        }));
        // enter not-foo
        new Trigger(() -> !foo).onTrue(print("C"));
        // exit not-foo
        new Trigger(() -> !foo).onFalse(print("D"));

    }

    void bad() {
        new Trigger(() -> foo).and(m_controller::getXButton)
                .onTrue(parallel(runOnce(() -> {
                    foo = false;
                }), print("A")))
                // onFalse never fires, because its 'previous' condition was false, and the
                // onTrue handler above resets it to false before onFalse handler sees the
                // true state.
                .onFalse(print("B"));
        new Trigger(() -> !foo).and(m_controller::getYButton)
                // here onFalse fires, because it sees the true transition
                .onFalse(print("D"))
                .onTrue(parallel(runOnce(() -> {
                    foo = true;
                }), print("C")));
    }

}
