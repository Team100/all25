package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetAlgaeDescorePositionPrep extends SequentialCommandGroup {
    public SetAlgaeDescorePositionPrep(LoggerFactory parent, Wrist2 wrist, Elevator elevator) {
        addCommands(
        // new RunAlgaeGrip(grip, 1)
        // new SetWrist(wrist, 0.5, false),
        // new SetElevator(elevator, 12, false),
        // new ParallelDeadlineGroup100(parent, new SetWrist(wrist, 3.7, false), new
        // SetElevatorPerpetually(elevator, 12) )

        );

        // addCommands(new SetWrist(wrist, 1.25, true));
    }
}
