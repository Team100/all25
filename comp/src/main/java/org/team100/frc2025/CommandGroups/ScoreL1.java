
package org.team100.frc2025.CommandGroups;

import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class ScoreL1 extends SequentialCommandGroup100 {
    public ScoreL1(LoggerFactory parent) {
        super(parent, "ScoreL1");
        addCommands();
    }
}
