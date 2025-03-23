package org.team100.frc2025.CommandGroups.Hesitant;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class ScoreL4Hesitantly extends SequentialCommandGroup100 {
    public ScoreL4Hesitantly(LoggerFactory logger, Wrist2 wrist, Elevator elevator) {
        super(logger);
        addCommands(
                new ParallelDeadlineGroup100(logger.child("out"),
                        new SetElevator(elevator, 35, false),
                        new SetWrist(wrist, 1.25, true)),
                new ParallelDeadlineGroup100(logger.child("down"),
                        new SetElevator(elevator, 10, false),
                        new SetWrist(wrist, 0.5, true)));
    }
}
