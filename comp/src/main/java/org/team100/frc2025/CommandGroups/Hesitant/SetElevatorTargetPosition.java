package org.team100.frc2025.CommandGroups.Hesitant;

import java.util.function.Supplier;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;

import edu.wpi.first.wpilibj2.command.Command;

public class SetElevatorTargetPosition extends Command {
    Elevator m_elevator;
    Supplier<ScoringPosition> positionSupplier;

    public SetElevatorTargetPosition(Elevator elevator, Supplier<ScoringPosition> pos) {
        m_elevator = elevator;
        positionSupplier = pos;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.setTargetScoringPosition(positionSupplier.get());
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
