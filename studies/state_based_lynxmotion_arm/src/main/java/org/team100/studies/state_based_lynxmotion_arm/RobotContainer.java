
package org.team100.studies.state_based_lynxmotion_arm;

import org.team100.lib.motion.lynxmotion_arm.LynxArmAngles;
import org.team100.studies.state_based_lynxmotion_arm.state.Alternative;
import org.team100.studies.state_based_lynxmotion_arm.subsystems.Arm;
import org.team100.studies.state_based_lynxmotion_arm.subsystems.ArmVisualization;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
    private final XboxController m_controller;
    private final Arm m_arm;
    // private final Chart m_chart;
    private final Alternative m_alt;
    private final ArmVisualization m_visualizer;

    public RobotContainer() {

        // calibrating servos.
        // don't use +/- pi/2 for calibration, it's in the saturation region.
        // these are for arm "ONE".
        // each arm is different. TODO: multiple configs.
        LynxArmAngles.Config config = new LynxArmAngles.Config();
        config.swingCenter = 0.47;
        config.swingScale = 3.2;
        config.boomCenter = 0.48;
        config.boomScale = 4;
        config.stickOffset = 0.05;
        config.stickScale = 3.4;
        config.wristCenter = 0.55;
        config.wristScale = 3.3;

        LynxArmAngles.Factory factory = new LynxArmAngles.Factory(config);

        m_controller = new XboxController(0);
        m_arm = new Arm(factory);
        // m_chart = new Chart(m_controller, m_arm);
        m_alt = new Alternative(m_controller, m_arm);
        m_visualizer = new ArmVisualization(m_arm);
        m_visualizer.start();
    }

    public void runChart() {
        // m_chart.poll();
        m_alt.poll();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
