package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CheckWristDanger;
import org.team100.frc2025.Wrist.IntakeAlgaeGrip;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class GrabAlgaeL3Dumb extends SequentialCommandGroup100 {
    public GrabAlgaeL3Dumb(
            LoggerFactory parent,
            Wrist2 wrist,
            Elevator elevator,
            AlgaeGrip grip) {
        super(parent);
        addCommands(
                // new SetAlgaeDescorePositionPrep(wrist, elevator),
                new CheckWristDanger(wrist),
                new ParallelDeadlineGroup100(parent,
                        new SetElevator(elevator, 35, true),
                        new SetWrist(wrist, 3.7, true),
                        new IntakeAlgaeGrip(grip, true)));
    }
}
