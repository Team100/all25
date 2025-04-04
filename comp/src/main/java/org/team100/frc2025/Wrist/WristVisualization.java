package org.team100.frc2025.Wrist;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.Logging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristVisualization implements Runnable {
    private final Wrist2 m_wrist;
    private final Mechanism2d m_mechanism;
    private final MechanismLigament2d m_ligament;

    public WristVisualization(Wrist2 wrist) {
        m_wrist = wrist;
        m_mechanism = new Mechanism2d(100, 100);
        MechanismRoot2d root = m_mechanism.getRoot("root", 50, 50);
        m_ligament = new MechanismLigament2d("wrist", 30, 0);
        var l2 = new MechanismLigament2d("cage", 20, 80);
        m_ligament.append(l2);
        root.append(m_ligament);
        SmartDashboard.putData("Wrist", m_mechanism);
    }

    @Override
    public void run() {
        if (Logging.instance().getLevel().admit(Level.TRACE)) {
            double position = m_wrist.getAngle();
            m_ligament.setAngle(new Rotation2d(1.6 - 1.1 * position));
        }
    }
}
