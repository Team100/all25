package org.team100.frc2025.CommandGroups;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class DeadlineForEmbarkAndPrePlace extends Command {
    Supplier<Boolean> m_embarkEnd;
    Supplier<Boolean> m_prePlaceEnd;

    boolean finished = false;

    public DeadlineForEmbarkAndPrePlace(Supplier<Boolean> embarkEnd, Supplier<Boolean> prePlaceEnd) {
        m_embarkEnd = embarkEnd;
        m_prePlaceEnd = prePlaceEnd;
    }

    @Override
    public void initialize() {
        finished = false;

        if (m_embarkEnd.get() && m_prePlaceEnd.get()) {
            finished = true;
        } else {
            finished = false;
        }
    }

    @Override
    public void execute() {
        if (m_embarkEnd.get() && m_prePlaceEnd.get()) {
            finished = true;
        } else {
            finished = false;
        }

    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println("DEADLINE FINISHED");
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
