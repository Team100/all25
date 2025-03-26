package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelCommandGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class ApproachL4 extends SequentialCommandGroup100 {

    public ApproachL4(LoggerFactory logger, Wrist2 wrist, Elevator elevator) {
        super(logger, "ApproachL4");
        addCommands(new SetWrist(wrist, 0.4, false),
                // new PrePlaceCoralL4(wrist, elevator, 45),
                // new ParallelDeadlineGroup100(parent,
                // new SetElevator(elevator, 45, false),
                // new SetWrist(wrist, 0.4, true)),//45
                new ParallelCommandGroup100(m_logger, "up",
                        new SetWrist(wrist, 1.25, false),
                        new SetElevator(m_logger,elevator, 45,false)));
    }
}
