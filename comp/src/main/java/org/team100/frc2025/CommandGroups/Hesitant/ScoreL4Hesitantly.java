package org.team100.frc2025.CommandGroups.Hesitant;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreL4Hesitantly extends SequentialCommandGroup {
    public ScoreL4Hesitantly(LoggerFactory parent, Wrist2 wrist, Elevator elevator) {
        addCommands(
                new ParallelDeadlineGroup100(parent,
                        new SetElevator(elevator, 35, false),
                        new SetWrist(wrist, 1.25, true)),
                new ParallelDeadlineGroup100(parent,
                        new SetElevator(elevator, 10, false),
                        new SetWrist(wrist, 0.5, true)));
    }
}
