package org.team100.lib.examples.motion;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

/**
 * This illustrates a short sequence, contained within a separate file, instead
 * of inlined in the Robot Container. If you put *everything* in the container,
 * it can get unwieldy, so making little classes like this helps keep things
 * tidy.
 */
public class SimpleSequence extends SequentialCommandGroup100 {
    public SimpleSequence(
            LoggerFactory log,
            RotaryPositionSubsystem1d rotary,
            OpenLoopSubsystem openloop) {
        super(log, "simple sequence",
                rotary.goToTheSpot().until(rotary::isDone),
                new ParallelDeadlineGroup100(log, "roll",
                        waitSeconds(2),
                        openloop.forward()),
                rotary.goHome().until(rotary::isDone));
    }
}
