package org.team100.frc2025.Funnel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class RunFunnel extends Command {
    Funnel m_funnel;
    Timer timer = new Timer();

    public RunFunnel(Funnel funnel) {
        m_funnel = funnel;
        addRequirements(m_funnel);
    }

    @Override
    public void initialize() {
        timer.restart();

    }

    @Override
    public void execute() {
        if (timer.get() < 0.75) {
            m_funnel.setFunnel(0.70);
        } else if (timer.get() < 0.85) {
            m_funnel.setFunnel(-0.2);
        } else {
            timer.restart();
        }

        // m_funnel.setFunnel(1);

    }

    @Override
    public void end(boolean interrupted) {
        m_funnel.setFunnel(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
