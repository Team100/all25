package org.team100.lib.examples.motion;

import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This is an example of what you'd put in RobotContainer to use some of the
 * subsystem and command classes here.
 */
public class ContainerSnippet {

    public ContainerSnippet() {
        // See the real RobotContainer for control setup.
        DriverControl control = new DriverControl() {
        };
        // See the real RobotContainer for log setup.
        LoggerFactory log = Logging.instance().rootLogger;

        // Creates a rotary positional subsystem.
        RotaryPositionSubsystem1d rotary = new RotaryPositionSubsystem1d(log);

        // Creates an open-loop subsystem
        OpenLoopSubsystem openloop = new OpenLoopSubsystem(log);

        /*
         * Binds the The trigger moves the subsystem to 1.5 with a profile.
         * 
         * Notice the pattern here:
         * 
         * * new trigger with a boolean, usually a controller button.
         * * "whileTrue" means run the command while you're holding the button
         * * the subsystem produces the command, which runs forever
         * * add a stop condition in "until"
         * 
         * Using an endless command with a separate stop condition is usually a good
         * choice: it avoids entangling the "stop" logic with the command itself. Note
         * in this case there's not even a Command class, it's just inline.
         * 
         * Note the use of "whileTrue" -- it cancels the command if you let go of the
         * button, which seems like a good idea.
         */
        new Trigger(control::trigger).whileTrue(rotary.goToTheSpot().until(rotary::isDone));

        /**
         * Binds some buttons to the open loop system.
         */
        new Trigger(control::button4).whileTrue(openloop.forward());
        new Trigger(control::button5).whileTrue(openloop.reverse());

        /**
         * Illustrates a short sequence: move, roll for 2 sec, move back.
         */
        new Trigger(control::feedFunnel).whileTrue(
                rotary.goToTheSpot().until(rotary::isDone)
                        .andThen(deadline(
                                waitSeconds(2),
                                openloop.forward()))
                        .andThen(rotary.goHome().until(rotary::isDone)));

        /**
         * This is another way to do the same thing, using a separate class to keep the
         * container tidy.
         */
        new Trigger(control::feedFunnel).whileTrue(new SimpleSequence(log, rotary, openloop));
    }
}
