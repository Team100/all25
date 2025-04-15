package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.state.Model100;

import edu.wpi.first.wpilibj2.command.Command;

public class SetElevatorBarge extends Command {
    private final Elevator m_elevator;
    private final IncrementalProfileReference1d m_ref;

    public SetElevatorBarge(Elevator elevator) {
        m_elevator = elevator;
        m_ref = elevator.defaultReference(0);

    }

    @Override
    public void initialize() {
        m_ref.init(new Model100(m_elevator.getPosition(), 0));
    }

    @Override
    public void execute() {
        // m_elevator.setPosition(0);
        m_elevator.setPositionSetpoint(m_ref.get());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
