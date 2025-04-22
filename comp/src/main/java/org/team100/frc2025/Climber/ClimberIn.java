package org.team100.frc2025.Climber;

import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberIn extends Command {
    Climber m_climber;

    public ClimberIn(Climber climber) {
        m_climber = climber;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        Profile100 profile100 = new TrapezoidProfile100(0.5, 0.5, 0.05);
        m_climber.setProfile(profile100);
        m_climber.reset();
    }

    @Override
    public void execute() {
        m_climber.setAngle(0.53);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
