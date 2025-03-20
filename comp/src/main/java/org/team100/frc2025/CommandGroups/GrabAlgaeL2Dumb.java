package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CheckWristDanger;
import org.team100.frc2025.Wrist.IntakeAlgaeGrip;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GrabAlgaeL2Dumb extends SequentialCommandGroup {
    public GrabAlgaeL2Dumb(LoggerFactory parent, Wrist2 wrist, Elevator elevator, AlgaeGrip grip) {
        addCommands(
                // new SetAlgaeDescorePositionPrep(wrist, elevator),
                new CheckWristDanger(wrist),
                new ParallelDeadlineGroup100(parent,
                        new SetElevator(elevator, 23, true),
                        new SetWrist(wrist, 3.7, true),
                        new IntakeAlgaeGrip(grip, true)));
    }
}
