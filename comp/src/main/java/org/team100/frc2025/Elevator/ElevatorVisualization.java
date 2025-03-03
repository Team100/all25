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
    private static final double kScale = 2.0;

    private final Elevator m_elevator;
    private final Mechanism2d m_mechanism;
    private final MechanismLigament2d m_base;
    // the elevator stages operate with their own roots
    MechanismRoot2d stage1root;
    private final MechanismLigament2d m_stage1;
    MechanismRoot2d stage2root;
    private final MechanismLigament2d m_stage2;
    MechanismRoot2d carriageRoot;
    private final MechanismLigament2d m_carriage;
    // the wrist is attached to the carriage
    private final MechanismLigament2d m_wrist;

    public ElevatorVisualization(Elevator elevator) {
        m_elevator = elevator;
        m_mechanism = new Mechanism2d(100, 100);
        MechanismRoot2d root = m_mechanism.getRoot("root", 50, 5);
        m_base = new MechanismLigament2d(
                "height",
                35,
                90,
                5,
                new Color8Bit(Color.kOrange));
        root.append(m_base);
        stage1root = m_mechanism.getRoot("stage1root", 45, 5);
        m_stage1 = new MechanismLigament2d("stage1", 35, 90, 5, new Color8Bit(Color.kOrange));
        stage1root.append(m_stage1);

        stage2root = m_mechanism.getRoot("stage2root", 40, 5);
        m_stage2 = new MechanismLigament2d("stage2", 35, 90, 5, new Color8Bit(Color.kOrange));
        stage2root.append(m_stage2);

        carriageRoot = m_mechanism.getRoot("carriageRoot", 35, 5);
        m_carriage = new MechanismLigament2d("carriage", 10, 90, 5, new Color8Bit(Color.kOrange));
        carriageRoot.append(m_carriage);

        m_wrist = new MechanismLigament2d("wrist", 10, 90, 5, new Color8Bit(Color.kOrange));
        m_carriage.append(m_wrist);

        SmartDashboard.putData("Elevator", m_mechanism);
    }

    public void viz() {
        if (Logging.instance().getLevel().admit(Level.TRACE)) {
            double position = m_elevator.getPosition();
            double stage1Position = Math.max(5, kScale * (position - 23.5));
            stage1root.setPosition(45, stage1Position);
            double stage2Position = Math.max(5, kScale * (position - 10));
            stage2root.setPosition(40, stage2Position);
            // carriage is always at the measured position
            carriageRoot.setPosition(35, kScale * (position) + 5);
        }
    }

}
