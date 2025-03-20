package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Funnel.RunFunnel;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.RunCoralTunnel;
import org.team100.frc2025.Wrist.SetWristDutyCycle;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RunFunnelHandoff extends SequentialCommandGroup {
    public RunFunnelHandoff(
            LoggerFactory parent,
            Elevator elevator,
            Wrist2 wrist,
            Funnel funnel,
            CoralTunnel tunnel,
            AlgaeGrip grip) {
        addCommands(
                new PrepareFunnelHandoff(parent, wrist, elevator),
                new ParallelCommandGroup100(parent,
                        new RunFunnel(funnel),
                        new RunCoralTunnel(tunnel, 1),
                        new SetWristDutyCycle(wrist, -0.15, false)));
    }
}
