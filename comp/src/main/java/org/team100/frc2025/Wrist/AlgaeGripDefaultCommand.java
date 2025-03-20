package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeGripDefaultCommand extends Command {
    AlgaeGrip m_grip;

    public AlgaeGripDefaultCommand(AlgaeGrip grip) {
        m_grip = grip;
        addRequirements(m_grip);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!m_grip.hasAlgae()) {
            m_grip.setDutyCycle(0);
        } else {
            m_grip.setDutyCycle(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
