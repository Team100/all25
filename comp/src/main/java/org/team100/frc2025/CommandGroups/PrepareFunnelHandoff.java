package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.SetWristHandoff;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelCommandGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class PrepareFunnelHandoff extends SequentialCommandGroup100 {

    public PrepareFunnelHandoff(LoggerFactory logger, Wrist2 wrist, Elevator elevator) {
        super(logger, "PrepareFunnelHandoff");

        addCommands(
                new ParallelCommandGroup100(m_logger, "prep",
                        // new SetWrist(wrist, 0.1, false),
                        new SetWristHandoff(wrist, 0.1),
                        new SetElevatorFunnelHandoff(elevator, 0.1))
        // new ParallelRaceGroup100(new WaitCommand(0.5), new
        // ElevatorDutyCycle(elevator)),
        // new SetWristHandoff(wrist, 0.1),
        // new ParallelRaceGroup100(new WaitCommand(0.2), new SetWristDutyCycle(wrist,
        // -0.15, false))

        );
    }
}
