package org.team100.frc2025.Elevator;

import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class ScoreAlgae extends SequentialCommandGroup100 {
    
    public ScoreAlgae(LoggerFactory parent, Wrist2 wrist, Elevator elevator, AlgaeGrip grip) {
        super(parent);

        addCommands(
        // new RunAlgaeGrip(grip, RunAlgaeGrip.GripState.INTAKE)
        // new SetWrist(wrist, 0.5, false),
        // new SetElevator(elevator, 55, false),
        // new ParallelDeadlineGroup100(parent, new SetWrist(wrist, 3.0, true), new
        // SetElevatorPerpetually(elevator, 57), new RunAlgaeGrip(grip, -1) ),
        // new ParallelDeadlineGroup100(parent, new SetWrist(wrist, 2.3, true), new
        // SetElevatorPerpetually(elevator, 57.2))
        // new ParallelDeadlineGroup100(parent, new SetElevator(elevator, 10, false),
        // new SetWrist(wrist, 1.25, true)),

        // new SetElevator(elevator, 2, false)

        // new ParallelDeadlineGroup100(parent, new SetElevator(elevator, 10, false),
        // new SetWrist(wrist, 1.26, false))
        // new SetWrist(wrist, 0.5, false)

        // new SetWrist(wrist, 0.5, false),
        // new SetElevator(elevator, 10, false),
        // new ParallelDeadlineGroup100(parent, new SetWrist(wrist, 3.7, true), new
        // SetElevatorPerpetually(elevator, 10) )

        );

        // addCommands(new SetWrist(wrist, 1.25, true));
    }
}
