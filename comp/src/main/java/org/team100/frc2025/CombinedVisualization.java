package org.team100.frc2025;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class CombinedVisualization implements Runnable {
    private static final double kScale = 2.0;

    private final Elevator m_elevator;
    private final Wrist2 m_wrist;
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
    private final MechanismLigament2d m_wristLigament;

    public CombinedVisualization(Elevator elevator, Wrist2 wrist) {
        m_elevator = elevator;
        m_wrist = wrist;
        m_mechanism = new Mechanism2d(100, 120);
        MechanismRoot2d root = m_mechanism.getRoot("combined_root", 70, 5);
        m_base = new MechanismLigament2d(
                "height",
                35,
                90,
                5,
                new Color8Bit(Color.kOrange));
        root.append(m_base);
        stage1root = m_mechanism.getRoot("combined_stage1root", 65, 5);
        m_stage1 = new MechanismLigament2d("stage1", 35, 90, 5, new Color8Bit(Color.kOrange));
        stage1root.append(m_stage1);

        stage2root = m_mechanism.getRoot("combined_stage2root", 60, 5);
        m_stage2 = new MechanismLigament2d("stage2", 35, 90, 5, new Color8Bit(Color.kOrange));
        stage2root.append(m_stage2);

        carriageRoot = m_mechanism.getRoot("combined_carriageRoot", 55, 5);
        m_carriage = new MechanismLigament2d("carriage", 10, 90, 5, new Color8Bit(Color.kOrange));
        carriageRoot.append(m_carriage);

        int wristLength = 40;
        m_wristLigament = new MechanismLigament2d("wrist", wristLength, 90, 5, new Color8Bit(Color.kOrange));
        m_carriage.append(m_wristLigament);

        SmartDashboard.putData("Combined", m_mechanism);
    }

    @Override
    public void run() {
        if (Logging.instance().getLevel().admit(Level.TRACE)) {
            double elevatorposition = m_elevator.getPosition();
            double stage1Position = Math.max(5, kScale * (elevatorposition - 23.5));
            stage1root.setPosition(65, stage1Position);
            double stage2Position = Math.max(5, kScale * (elevatorposition - 10));
            stage2root.setPosition(60, stage2Position);
            // carriage is always at the measured position
            carriageRoot.setPosition(55, kScale * (elevatorposition) + 5);
            double wristPosition = m_wrist.getAngle();
            m_wristLigament.setAngle(Math.toDegrees(wristPosition));
        }
    }
}
