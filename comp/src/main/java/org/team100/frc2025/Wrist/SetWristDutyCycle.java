package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SetWristDutyCycle extends Command {
    Wrist2 m_wrist;
    double m_duty;
    boolean m_withCount;
    Timer m_timer;
    boolean isDone = false;

    public SetWristDutyCycle(Wrist2 wrist, double duty, boolean withCount) {
        m_wrist = wrist;
        m_duty = duty;
        m_withCount = withCount;
        m_timer = new Timer();
        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        isDone = false;
    }

    @Override
    public void execute() {
        m_wrist.setWristDutyCycle(m_duty);

        if (m_timer.get() > 2) {
            isDone = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        isDone = false;

        // System.out.println("**************************************I FINISHED NUMBER
        // 4*******************************************");

    }

    @Override
    public boolean isFinished() {
        if (m_withCount) {
            return isDone;
        }
        return false;
    }
}
