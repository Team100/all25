package org.team100.lib.examples.motion;

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

        // Creates the subsystem.
        RotaryPositionSubsystem1d rotarySubsystem = new RotaryPositionSubsystem1d(log);

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
         */
        new Trigger(control::trigger).whileTrue(rotarySubsystem.setProfiled(1.5).until(rotarySubsystem::isDone));

    }

}
