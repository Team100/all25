package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class StopAlgaeGrip extends Command {
    AlgaeGrip m_grip;
    boolean m_perpetual = false;
    double position = 0;
    boolean hasAlgae = false;

    public StopAlgaeGrip(AlgaeGrip grip, boolean perpetual) {
        m_grip = grip;
        m_perpetual = perpetual;
        addRequirements(m_grip);
    }

    @Override
    public void initialize() {
        position = 0;
        hasAlgae = false;
    }

    @Override
    public void execute() {
        if (!m_grip.hasAlgae()) {
            m_grip.setDutyCycle(-1);
            // position = m_grip.getPosition();
        } else {
            hasAlgae = true;
        }

        if (hasAlgae) {
            // System.out.println("*************HAS ALGAE****************");
            // m_grip.setHoldPosition(position);
            // m_grip.setPosition(position);
            // m_grip.setDutyCycle(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_grip.setDutyCycle(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
