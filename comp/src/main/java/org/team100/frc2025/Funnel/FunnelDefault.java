package org.team100.frc2025.Funnel;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelDefault extends Command {
    Funnel m_funnel;

    public FunnelDefault(Funnel funnel) {
        m_funnel = funnel;
        addRequirements(m_funnel);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_funnel.setFunnel(0);
        m_funnel.setLatch1(0);
        m_funnel.setLatch2(180);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
