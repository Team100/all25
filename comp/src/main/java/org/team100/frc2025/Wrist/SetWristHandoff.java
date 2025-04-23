package org.team100.frc2025.Wrist;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

import edu.wpi.first.wpilibj2.command.Command;

public class SetWristHandoff extends Command {
    private final Wrist2 m_wrist;
    private final double m_angle;
    private boolean finished = false;
    private double count = 0;

    public SetWristHandoff(Wrist2 wrist, double angle) {
        m_wrist = wrist;
        m_angle = angle;
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

        if (Math.abs(m_wrist.getAngle() - m_angle) < 0.3) {
            count++;
            m_wrist.setAngleValue(m_angle);

        } else {
            m_wrist.setAngleValue(m_angle);
            count = 0;
        }

        if (count >= 2) {
            // m_wrist.setStatic();
            finished = true;
        }

        // if(m_wrist.getAngle() < m_angle){
        // finished = true;
        // }

    }

    @Override
    public void end(boolean interrupted) {
        m_wrist.setWristDutyCycle(0);
        finished = false;
        count = 0;
    }

    @Override
    public boolean isFinished() {
        if (Experiments.instance.enabled(Experiment.UseProfileDone))
            return finished && m_wrist.profileDone();
        return finished;
    }
}
