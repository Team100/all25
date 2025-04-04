package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Funnel.RunFunnel;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CheckFunnelDanger;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.RunCoralTunnel;
import org.team100.frc2025.Wrist.SetWristDutyCycle;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelCommandGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class RunFunnelHandoff extends SequentialCommandGroup100 {
    public RunFunnelHandoff(
            LoggerFactory logger,
            Elevator elevator,
            Wrist2 wrist,
            Funnel funnel,
            CoralTunnel tunnel,
            AlgaeGrip grip) {
        super(logger, "RunFunnelHandoff");
        addCommands(
                new CheckFunnelDanger(wrist, elevator),
                new PrepareFunnelHandoff(m_logger, wrist, elevator),
                new ParallelCommandGroup100(m_logger, "handoff",
                        new RunFunnel(funnel),
                        new RunCoralTunnel(tunnel, 1),
                        new SetWristDutyCycle(wrist, -0.15, false)));
    }
}
