package org.team100.frc2025.Wrist;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

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
        // resetting forces the setpoint velocity to zero, which is not always what we
        // want
        // m_wrist.resetWristProfile();
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
            // i think "set static" was an attempt to push gently into the funnel, but this
            // is now accomplished with a different command which sets a duty cycle.
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
