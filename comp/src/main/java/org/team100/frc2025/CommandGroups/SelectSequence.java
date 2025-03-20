package org.team100.frc2025.CommandGroups;

import java.util.function.Supplier;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;

import edu.wpi.first.wpilibj2.command.Command;

public class SelectSequence extends Command {

    Command m_l1Command;
    Command m_l2Commad;
    Command m_l3Command;
    Command m_l4Command;
    Wrist2 m_wrist;
    Elevator m_elevator;
    Supplier<ScoringPosition> m_scoringSupplier;
    ScoringPosition m_position = ScoringPosition.NONE;

    public SelectSequence(Supplier<ScoringPosition> scoringSupplier, Command l1Command, Command l2Commad,
            Command l3Command, Command l4Command, Wrist2 wrist, Elevator elevator) {

        m_l1Command = l1Command;
        m_l2Commad = l2Commad;
        m_l3Command = l3Command;
        m_l4Command = l4Command;
        m_wrist = wrist;
        m_elevator = elevator;
        m_scoringSupplier = scoringSupplier;
    }

    @Override
    public void initialize() {
        m_position = m_scoringSupplier.get();
    }

    @Override
    public void execute() {
        switch (m_position) {
            case L4:
                m_l4Command.execute();
                break;
            case L3:
                m_l3Command.execute();
                break;
            case L2:
                m_l2Commad.execute();
                break;
            case L1:
                m_l1Command.execute();
                break;
            case NONE:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
