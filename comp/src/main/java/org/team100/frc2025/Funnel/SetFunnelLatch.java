package org.team100.frc2025.Funnel;

import edu.wpi.first.wpilibj2.command.Command;

public class SetFunnelLatch extends Command {
    Funnel m_funnel;
    double m_value1;
    double m_value2;

    public SetFunnelLatch(Funnel funnel, double value1, double value2) {
        m_funnel = funnel;
        m_value1 = value1;
        m_value2 = value2;
        addRequirements(m_funnel);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_funnel.setLatch1(m_value1);
        m_funnel.setLatch2(m_value2);

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // double error1 = Math.abs(m_funnel.getLatch1() - m_value1);
        // double error2 = Math.abs(m_funnel.getLatch2() - m_value2);

        // if(error1 < 3 && error2 < 3){
        // return true;
        // }

        // if(error1 < 3){
        // return true;
        // }

        return false;
    }
}
