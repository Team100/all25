package org.team100.frc2025.CommandGroups;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class WaitForBooleanTrue extends Command {
    Supplier<Boolean> m_booleanSupplier;
    boolean finished = false;

    public WaitForBooleanTrue(Supplier<Boolean> booleanSupplier) {
        m_booleanSupplier = booleanSupplier;
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        if (m_booleanSupplier.get()) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(" I THINK YOURE TRUE ");
        finished = false;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
