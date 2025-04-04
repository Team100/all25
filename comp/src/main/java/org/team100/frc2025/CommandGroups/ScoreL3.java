package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class ScoreL3 extends SequentialCommandGroup100 {

    public ScoreL3(LoggerFactory logger, Wrist2 wrist, Elevator elevator) {
        super(logger, "ScoreL3");
        addCommands(
                new SetWrist(wrist, 0.4, false),
                new PrePlaceCoralL3(wrist, elevator, 23, false),
                // new ParallelDeadlineGroup100(parent,
                // new SetElevator(elevator, 23, false), new
                // SetWrist(wrist, 0.4, true)),
                new ParallelDeadlineGroup100(m_logger, "up",
                        new SetWrist(wrist, 0.9, false),
                        new SetElevatorPerpetually(elevator, 23)),
                new ParallelDeadlineGroup100(m_logger, "down",
                        new SetElevator(m_logger, elevator, 16, false),
                        new SetWrist(wrist, 0.9, true)),
                new ParallelDeadlineGroup100(m_logger, "in",
                        new SetElevator(m_logger, elevator, 10, false),
                        new SetWrist(wrist, 0.5, true))

        // new ParallelDeadlineGroup100(parent,
        // new SetElevator(elevator, 35, false), new
        // SetWrist(wrist, 1.25, true))
        );
    }
}
