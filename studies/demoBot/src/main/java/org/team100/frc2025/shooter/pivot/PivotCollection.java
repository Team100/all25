package org.team100.frc2025.shooter.pivot;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550CANSparkMotor;

public class PivotCollection {
    private static final String kPivot = "Pivot";

    private final Neo550CANSparkMotor m_pivot;
    private static final double m_5to1 = 5.2307692308;
    private static final double m_4to1 = 3.61;

    private PivotCollection(Neo550CANSparkMotor pivot) {
        m_pivot = pivot;
    }

    public static PivotCollection get(
            LoggerFactory parent,
            int currentLimit
            ) {
        LoggerFactory collectionLogger = parent.child(kPivot);
        switch (Identity.instance) {
            case DEMO_BOT:
            //TODO get canID, gearRatio, p values, and gravityNm
            Neo550CANSparkMotor pivot = new Neo550CANSparkMotor(collectionLogger, 5, MotorPhase.FORWARD, currentLimit, Feedforward100.makeNeo550(), new PIDConstants());
            return new PivotCollection(pivot);
            case BLANK:
            default:
            throw new UnsupportedOperationException("Not correct robot");
        }
    }

    public Neo550CANSparkMotor getPivot() {
        return m_pivot;
    }
}
