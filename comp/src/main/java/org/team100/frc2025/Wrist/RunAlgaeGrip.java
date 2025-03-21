package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class RunAlgaeGrip extends Command {
    AlgaeGrip m_grip;

    public RunAlgaeGrip(AlgaeGrip tunnel) {
        m_grip = tunnel;
        addRequirements(m_grip);
    }

    @Override
    public void initialize() {
        m_grip.applyHighConfigs();
        m_grip.setDutyCycle(1);
    }

    @Override
    public void execute() {
        if(m_grip.hasAlgae()){
            m_grip.applyLowConfigs();
        }
        // m_grip.intake();
    }

    public void end(boolean interrupted) {
        m_grip.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
