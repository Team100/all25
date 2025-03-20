package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Wrist.AlgaeGrip;

import edu.wpi.first.wpilibj2.command.Command;

public class StopAlgaeGrip extends Command {
    AlgaeGrip m_grip;

    public StopAlgaeGrip(AlgaeGrip grip) {
        m_grip = grip;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_grip.setDutyCycle(0);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
