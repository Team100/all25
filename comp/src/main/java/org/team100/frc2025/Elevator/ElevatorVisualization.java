package org.team100.frc2025.Elevator;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorVisualization {
    private static final double kScale = 10.0;

    private final Elevator m_elevator;
    private final Mechanism2d m_mechanism;
    private final MechanismLigament2d m_ligament;

    public ElevatorVisualization(Elevator elevator) {
        m_elevator = elevator;
        m_mechanism = new Mechanism2d(100, 100);
        MechanismRoot2d root = m_mechanism.getRoot("root", 50, 50);
        double position = m_elevator.getPosition();
        m_ligament = new MechanismLigament2d("height", kScale * position + 10, 90, 5, new Color8Bit(Color.kOrange));
        root.append(m_ligament);
        SmartDashboard.putData("Elevator", m_mechanism);
    }

    public void viz() {
        if (Logging.instance().getLevel().admit(Level.TRACE)) {
            double position = m_elevator.getPosition();
            m_ligament.setLength(kScale * position + 10);
        }
    }

}
