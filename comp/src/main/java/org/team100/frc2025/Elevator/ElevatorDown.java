
package org.team100.frc2025.Elevator;

import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Model100;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorDown extends Command {
    private final Elevator m_elevator;
    private final IncrementalProfileReference1d m_ref;

    public ElevatorDown(Elevator elevator) {
        m_elevator = elevator;
        m_ref = elevator.defaultReference(0.2286);
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        // m_elevator.resetProfile();
        m_ref.init(new Model100(m_elevator.getPosition(), 0));
    }

    @Override
    public void execute() {
        // m_elevator.setDutyCycle(-0.05);
        m_elevator.setPositionSetpoint(m_ref.get());
        // m_elevator.setPosition(0.2286); // 16 for l3
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.setDutyCycle(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
