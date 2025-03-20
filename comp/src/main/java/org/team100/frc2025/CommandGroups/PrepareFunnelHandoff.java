package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.SetWristHandoff;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PrepareFunnelHandoff extends SequentialCommandGroup {
    private final Wrist2 m_wrist;
    private final Elevator m_elevator;

    public PrepareFunnelHandoff(LoggerFactory parent, Wrist2 wrist, Elevator elevator) {
        m_wrist = wrist;
        m_elevator = elevator;
        addCommands(
                new ParallelCommandGroup100(
                        parent,
                        // new SetWrist(wrist, 0.1, false),
                        new SetWristHandoff(wrist, 0.1),
                        new SetElevatorFunnelHandoff(elevator, 0.1))
        // new ParallelRaceGroup(new WaitCommand(0.5), new ElevatorDutyCycle(elevator)),
        // new SetWristHandoff(wrist, 0.1),
        // new ParallelRaceGroup(new WaitCommand(0.2), new SetWristDutyCycle(wrist,
        // -0.15, false))

        );
    }
}
