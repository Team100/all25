package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.PathPoint;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.tuning.Mutable;

/** Trivial constraint for testing. */
public class ConstantConstraint implements TimingConstraint {
    private final Mutable m_maxVelocity;
    private final Mutable m_maxAccel;

    public ConstantConstraint(LoggerFactory parent, double maxV, double maxA) {
        LoggerFactory log = parent.type(this);
        m_maxVelocity = new Mutable(log, "maxV", maxV);
        m_maxAccel = new Mutable(log, "maxA", maxA);
    }

    public ConstantConstraint(LoggerFactory log, double vScale, double aScale, SwerveKinodynamics limits) {
        this(log, vScale * limits.getMaxDriveVelocityM_S(), aScale * limits.getMaxDriveAccelerationM_S2());
    }

    @Override
    public double maxV(PathPoint state) {
        return m_maxVelocity.getAsDouble();
    }

    @Override
    public double maxAccel(PathPoint state, double velocityM_S) {
        return m_maxAccel.getAsDouble();
    }
    
    @Override
    public double maxDecel(PathPoint state, double velocity) {
        return -m_maxAccel.getAsDouble();
    }
}
