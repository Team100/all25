package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class SetWristHandoff extends Command {
    Wrist2 m_wrist;
    double m_angle;
    boolean finished = false;
    double count = 0;

    public SetWristHandoff(Wrist2 wrist, double angle) {
        m_wrist = wrist;
        m_angle = angle;
        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        finished = false;
        count = 0;
        m_wrist.resetWristProfile();
    }

    @Override
    public void execute() {

        if (Math.abs(m_wrist.getAngle() - m_angle) < 0.05) {
            count++;
            m_wrist.setAngleValue(m_angle);

        } else {
            m_wrist.setAngleValue(m_angle);
            count = 0;
        }

        if (count >= 20) {
            m_wrist.setStatic();
            finished = true;
        }

        // if(m_wrist.getAngle() < m_angle){
        // finished = true;
        // }

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
        // return m_wrist.atSetpoint();

        return finished;
    }
}
