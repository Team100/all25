package org.team100.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that executes a motion and then holds the ending position. The
 * method isDone() returns true while holding. This is useful for parallel
 * command compositions, so that some commands can be "done" early, while
 * holding their ending positions, waiting for the rest of the commands to be
 * done.
 * 
 * The same thing is not possible using the WPI isFinished() method, because the
 * members of the composition will end, no longer holding their end positions,
 * but their required subsystems will not be released, so their default commands
 * won't be running either, leaving the "early finishing" subsystems completely
 * uncontrolled.
 * 
 * This extends Command so you can use this type in compositions.
 */
public abstract class MoveAndHold extends Command {
    /**
     * True if we've started and we're finished.
     * Note: after end, isDone will yield false.
     */
    public abstract boolean isDone();
}
