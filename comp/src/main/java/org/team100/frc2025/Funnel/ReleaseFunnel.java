
package org.team100.frc2025.Funnel;

import org.team100.frc2025.Climber.Climber;
import org.team100.frc2025.Climber.SetClimber;
import org.team100.lib.framework.ParallelRaceGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ReleaseFunnel extends SequentialCommandGroup100 {
    public ReleaseFunnel(LoggerFactory logger, Funnel funnel, Climber climber) {
        super(logger, "ReleaseFunnel");
        addCommands(
                new ParallelRaceGroup100(m_logger, "unlatch",
                        new SetFunnelLatch(funnel, 180, 0),
                        new WaitCommand(1)),
                        
                new SetClimber(climber, 2.51)
                
        );

                // new ParallelRaceGroup100(m_logger, "emerge",
                //         new SetClimber(climber, -1.42),
                //         new WaitCommand(1)));
    }
}
