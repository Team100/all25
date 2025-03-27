package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class CheckWristDanger extends Command {
    Wrist2 m_wrist;
    double m_angle;
    boolean finished = false;
    double count = 0;

    public CheckWristDanger(Wrist2 wrist) {
        m_wrist = wrist;
        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        finished = false;
        count = 0;
        // resetting forces the setpoint velocity to zero, which is not always what we
        // want
        // m_wrist.resetWristProfile();
    }

    @Override
    public void execute() {
        double m_angle = 0.1;

        if (m_wrist.getAngle() < m_angle) {
            m_wrist.setAngleValue(m_angle);

            if (Math.abs(m_wrist.getAngle() - m_angle) < 0.05) {
                count++;
            } else {
                count = 0;
            }

        } else {
            finished = true;
        }

        if (count >= 20) {
            finished = true;
        }

    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println("**************************************I
        // FINISHED*******************************************");
        m_wrist.setWristDutyCycle(0);
        finished = false;
        count = 0;
    }

    @Override
    public boolean isFinished() {
        // return m_wrist.atSetpoint()
        return finished;
    }
}
