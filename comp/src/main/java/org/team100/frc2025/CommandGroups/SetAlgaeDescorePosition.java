
package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class SetAlgaeDescorePosition extends SequentialCommandGroup100 {

    public SetAlgaeDescorePosition(LoggerFactory parent, Wrist2 wrist, Elevator elevator) {
        super(parent, "SetAlgaeDescorePosition");
        addCommands(

        );

    }
}
