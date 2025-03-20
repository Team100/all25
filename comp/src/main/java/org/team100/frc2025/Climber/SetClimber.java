package org.team100.frc2025.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class SetClimber extends Command {
    Climber m_climber;
    double m_value;

    public SetClimber(Climber climber, double value) {
        m_climber = climber;
        m_value = value;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.reset();
    }

    @Override
    public void execute() {
        m_climber.setAngle(m_value);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
