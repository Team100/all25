package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class SetWrist extends Command {
    private final Wrist2 m_wrist;
    private final double m_angle;
    private final boolean m_perpetual;
    private boolean finished = false;
    private double count = 0;

    public SetWrist(Wrist2 wrist, double angle, boolean perpetual) {
        m_wrist = wrist;
        m_angle = angle;
        m_perpetual = perpetual;
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

        if (!m_perpetual) {
            if (Math.abs(m_wrist.getAngle() - m_angle) < 0.05) {
                count++;
                m_wrist.setAngleValue(m_angle);
            } else {
                {
                    m_wrist.setAngleValue(m_angle);
                    count = 0;
                }
            }

            if (count >= 5) {
                finished = true;
            }
        } else {
            if (Math.abs(m_wrist.getAngle() - m_angle) < 0.05) {
                count++;
                m_wrist.setAngleValue(m_angle);

            } else {
                m_wrist.setAngleValue(m_angle);
                count = 0;
            }

            if (count >= 5) {
                m_wrist.setStatic();
            }
        }

        // m_wrist.setStatic();

        // m_wrist.setAngleValue(m_angle);

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
        if (m_perpetual) {
            return false;
        }
        return finished;
    }
}
