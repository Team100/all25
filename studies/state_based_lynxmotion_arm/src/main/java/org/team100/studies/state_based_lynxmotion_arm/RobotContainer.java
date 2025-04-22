
package org.team100.studies.state_based_lynxmotion_arm;

import org.team100.studies.state_based_lynxmotion_arm.state.Chart;
import org.team100.studies.state_based_lynxmotion_arm.subsystems.Arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
    private final XboxController m_controller;
    private final Arm m_arm;
    private final Chart m_chart;

    public RobotContainer() {
        m_controller = new XboxController(0);
        m_arm = new Arm();
        m_chart = new Chart(m_controller);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
