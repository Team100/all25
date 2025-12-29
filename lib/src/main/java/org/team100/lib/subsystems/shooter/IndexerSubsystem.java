package org.team100.lib.subsystems.shooter;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.servo.OutboardLinearPositionServo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    private final double m_IndexerVelocityM_S;
    private final double m_objectLength;

    private final LoggerFactory m_logger;
    private final OutboardLinearPositionServo m_indexer;
    private final LinearMechanism m_linearMechanism;

    public IndexerSubsystem(
            LoggerFactory parent, LinearMechanism linearMechanism, double maxAccel,
            double objectLengthM, double indexVelocityM_S) {
        m_logger = parent.type(this);
        m_objectLength = objectLengthM;
        m_IndexerVelocityM_S = indexVelocityM_S;
        m_linearMechanism = linearMechanism;
        TrapezoidIncrementalProfile profile = new TrapezoidIncrementalProfile(
                m_logger, indexVelocityM_S, maxAccel, 0.02);
        m_indexer = new OutboardLinearPositionServo(
                m_logger,
                linearMechanism,
                new IncrementalProfileReferenceR1(
                        m_logger,
                        () -> profile,
                        0.02,
                        0.02),
                0.02, 0.02);
    }

    public void index() {
        set(m_IndexerVelocityM_S);
    }

    public void unindex() {
        set(-1.0 * m_IndexerVelocityM_S);
    }

    public void indexOne() {
        setAngle(m_indexer.getPosition() + m_objectLength);
    }

    public void unindexOne() {
        setAngle(m_indexer.getPosition() - m_objectLength);
    }

    public void stop() {
        m_indexer.stop();
    }

    public double getVelocity() {
        return m_indexer.getVelocity();
    }

    public void set(double value) {
        m_linearMechanism.setVelocity(value, 0, 0);
    }

    public void setAngle(double value) {
        m_indexer.setPositionProfiled(value, 0);
    }

    public double getAngle() {
        return m_indexer.getPosition();
    }

    @Override
    public void periodic() {
        m_indexer.periodic();
    }
}
