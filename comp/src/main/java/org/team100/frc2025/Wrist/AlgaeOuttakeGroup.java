package org.team100.frc2025.Wrist;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AlgaeOuttakeGroup extends ParallelCommandGroup {
    public AlgaeOuttakeGroup(AlgaeGrip grip, Wrist2 wrist, Elevator elevator) {
        addCommands(
                new OuttakeAlgaeGrip(grip),
                new SetWrist(wrist, 3.7, true),
                new SetElevatorPerpetually(elevator, 12));
    }
}
