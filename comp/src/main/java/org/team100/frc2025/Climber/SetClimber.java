package org.team100.frc2025.Climber;

import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.state.Model100;

import edu.wpi.first.wpilibj2.command.Command;

public class SetClimber extends Command {
    private final Climber m_climber;
    // private final double m_value;
    private final IncrementalProfileReference1d m_ref;

    public SetClimber(Climber climber, double value) {
        m_climber = climber;
        // m_value = value;
        Profile100 profile100 = new TrapezoidProfile100(0.5, 0.5, 0.05);
        m_ref = new IncrementalProfileReference1d(profile100, new Model100(value, 0));
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.reset();
        m_ref.init(new Model100(m_climber.getAngle(), 0));
    }

    @Override
    public void execute() {
        // m_climber.setAngle(m_value);
        m_climber.setAngleSetpoint(m_ref.get());
    }
}
