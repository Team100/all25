package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class ApproachL3 extends SequentialCommandGroup100 {

    public ApproachL3(LoggerFactory logger, Wrist2 wrist, Elevator elevator) {
        super(logger, "ApproachL3");
        addCommands(new SetWrist(wrist, 0.4, false),
        new PrePlaceCoralL3(wrist, elevator, 23),
        // new ParallelDeadlineGroup100(parent,
        // new SetElevator(elevator, 23, false), new
        // SetWrist(wrist, 0.4, true)),
        new ParallelDeadlineGroup100(m_logger, "up",
                new SetWrist(wrist, 0.9, false),
                new SetElevatorPerpetually(elevator, 23)));
    }
}
