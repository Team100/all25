package org.team100.studies.state_based_lynxmotion_arm.subsystems;


import org.team100.lib.motion.lynxmotion_arm.LynxArmAngles;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Observes the arm position asynchronously and tells the dashboard. */
public class ArmVisualization {

    private final Arm m_arm;

    private final Mechanism2d m_topView = new Mechanism2d(100, 100);
    private final MechanismRoot2d m_topRoot = m_topView.getRoot("TopRoot", 50, 50);
    private final MechanismLigament2d m_swingLigament = new MechanismLigament2d(
            "Swing", 15, 90, 5, new Color8Bit(Color.kWhite));

    private final Mechanism2d m_sideView = new Mechanism2d(100, 100);
    private final MechanismRoot2d m_sideRoot = m_sideView.getRoot("SideRoot", 50, 50);
    private final MechanismLigament2d m_boomLigament = new MechanismLigament2d(
            "Boom", 20, 90, 5, new Color8Bit(Color.kWhite));
    private final MechanismLigament2d m_stickLigament = new MechanismLigament2d(
            "Stick", 20, -90, 5, new Color8Bit(Color.kLightGreen));
    private final MechanismLigament2d m_wristLigament = new MechanismLigament2d(
            "Wrist", 5, -90, 5, new Color8Bit(Color.kLightBlue));

    private final Notifier m_notifier = new Notifier(this::update);

    /** Visualize the arm position in the dashboard. */
    public ArmVisualization(Arm arm) {
        m_arm = arm;
        m_topRoot.append(m_swingLigament);
        SmartDashboard.putData("TopView", m_topView);
        m_sideRoot.append(m_boomLigament);
        m_boomLigament.append(m_stickLigament);
        m_stickLigament.append(m_wristLigament);
        SmartDashboard.putData("SideView", m_sideView);
    }

    /* Starts the notifier. */
    public void start() {
        m_notifier.startPeriodic(0.1); // 10Hz = slower than the robot loop
    }

    /* Reads current servo positions and writes them to the dashboard */
    public void update() {
        LynxArmAngles angles = m_arm.get();
        double swingLength = 20 * Math.cos(Math.PI * angles.boom)
                + 20 * Math.cos(Math.PI * (angles.boom - angles.stick));

        m_swingLigament.setAngle(180 * angles.swing);
        m_swingLigament.setLength(swingLength);
        m_boomLigament.setAngle(180 * angles.boom);
        m_stickLigament.setAngle(-180 * angles.stick);
        m_wristLigament.setAngle(180 * (angles.wrist));
    }

}