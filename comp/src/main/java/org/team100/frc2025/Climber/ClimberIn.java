package org.team100.frc2025.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberIn extends Command {
    Climber m_climber;

    public ClimberIn(Climber climber) {
        m_climber = climber;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
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
