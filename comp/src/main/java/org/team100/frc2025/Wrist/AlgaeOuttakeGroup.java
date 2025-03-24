package org.team100.frc2025.Wrist;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;
import org.team100.lib.framework.ParallelCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class AlgaeOuttakeGroup extends ParallelCommandGroup100 {
    public AlgaeOuttakeGroup(
            LoggerFactory logger,
            AlgaeGrip grip,
            Wrist2 wrist,
            Elevator elevator) {
        super(logger, "AlgaeOuttakeGroup");
        addCommands(
                new OuttakeAlgaeGrip(grip),
                new SetWrist(wrist, 3.7, true),
                new SetElevatorPerpetually(elevator, 12));
    }
}
