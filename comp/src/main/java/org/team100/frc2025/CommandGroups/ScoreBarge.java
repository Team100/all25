package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.IntakeAlgaeGrip;
import org.team100.frc2025.Wrist.OuttakeAlgaeGrip;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class ScoreBarge extends SequentialCommandGroup100 {
    public ScoreBarge(LoggerFactory logger, Elevator elevator, Wrist2 wrist, AlgaeGrip grip) {
        super(logger, "ScoreBarge");
        // addCommands(
        // // new ParallelDeadlineGroup100(
        // // parent,
        // // new SetElevator(elevator, 54, false),
        // // new SetWrist(wrist, 3.4, true),
        // // new IntakeAlgaeGrip(algae, true)
        // // // new RunAlgaeGrip(algae)
        // // ),
        // // new ParallelCommandGroup100(
        // // parent,
        // // new SetElevatorPerpetually(elevator, 54)
        // // // new OuttakeAlgaeGrip(wrist, algae)
        // // )

        // );
        // // new OuttakeAlgaeGrip(wrist, algae)

        addCommands(
                // new SetAlgaeDescorePositionPrep(wrist, elevator),
                // new CheckWristDanger(wrist),
                new ParallelDeadlineGroup100(m_logger, "intake",
                        new SetWrist(wrist, 2.0, false),
                        new SetElevator(elevator, 54, true),
                        new IntakeAlgaeGrip(grip, true)),
                new ParallelDeadlineGroup100(m_logger, "shoot",
                        new OuttakeAlgaeGrip(grip),
                        new SetWrist(wrist, 2.0, true),
                        new SetElevator(elevator, 54, true)

                )
        // new IntakeAlgaeGrip(grip, true))
        );
    }
}
