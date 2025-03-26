package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class ApproachL2 extends SequentialCommandGroup100 {

    public ApproachL2(LoggerFactory logger, Wrist2 wrist, Elevator elevator) {
        super(logger, "ApproachL2");
        addCommands(new SetWrist(wrist, 0.4, false),
                new ParallelDeadlineGroup100(m_logger, "up",
                        new SetElevator(m_logger, elevator, 10.5, false),
                        new SetWrist(wrist, 0.4, true)),
                new ParallelDeadlineGroup100(m_logger, "out",
                        new SetWrist(wrist, 0.9, false),
                        new SetElevatorPerpetually(elevator, 10.5)));
    }
}
