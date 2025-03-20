package org.team100.frc2025.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDefault extends Command {
    Climber m_climber;

    public ClimberDefault(Climber climber) {
        m_climber = climber;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_climber.setDutyCycle(0);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
