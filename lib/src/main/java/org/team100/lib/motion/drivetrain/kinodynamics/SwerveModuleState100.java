package org.team100.lib.motion.drivetrain.kinodynamics;

import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.struct.SwerveModuleStateStruct;
import edu.wpi.first.util.struct.StructSerializable;

/** The state of one swerve module. */
public class SwerveModuleState100 implements Comparable<SwerveModuleState100>, StructSerializable {
    private final double m_speedM_S;
    private final Optional<Rotation2d> m_angle;

    /** Zero speed and indeterminate angle. */
    public SwerveModuleState100() {
        this(0, Optional.empty());
    }

    public SwerveModuleState100(double speedM_S, Optional<Rotation2d> angle) {
        m_speedM_S = speedM_S;
        m_angle = angle;
    }

    /**
     * Returns empty angle if velocity is about zero.
     * Otherwise angle is always within [-pi, pi].
     */
    public static SwerveModuleState100 fromSpeed(double vx, double vy) {
        if (Math.abs(vx) < 0.004 && Math.abs(vy) < 0.004) {
            return new SwerveModuleState100(0.0, Optional.empty());
        } else {
            return new SwerveModuleState100(Math.hypot(vx, vy), Optional.of(new Rotation2d(vx, vy)));
        }
    }

    /**
     * Minimize the change in heading the desired swerve module state would require
     * by potentially reversing the direction the wheel spins. If this is used with
     * the PIDController class's continuous input functionality, the furthest a
     * wheel will ever rotate is 90 degrees.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     * @return Optimized swerve module state.
     */
    public static SwerveModuleState100 optimize(
            SwerveModuleState100 desiredState, Rotation2d currentAngle) {
        if (desiredState.m_angle.isEmpty()) {
            // this does happen
            return desiredState;
        }
        var delta = desiredState.m_angle.get().minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SwerveModuleState100(
                    -desiredState.m_speedM_S,
                    Optional.of(desiredState.m_angle.get().rotateBy(Rotation2d.fromDegrees(180.0))));
        } else {
            return new SwerveModuleState100(
                    desiredState.m_speedM_S,
                    desiredState.m_angle);
        }
    }

    /** Speed of the wheel of the module. */
    public double speedMetersPerSecond() {
        return m_speedM_S;
    }

    /**
     * Angle of the module. It can be empty, in cases where the angle is
     * indeterminate (e.g. calculating the angle required for zero speed).
     */
    public Optional<Rotation2d> angle() {
        return m_angle;
    }

    @Override
    public String toString() {
        return String.format(
                "SwerveModuleState(Speed: %.2f m/s, Angle: %s)",
                m_speedM_S, m_angle);
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_speedM_S, m_angle);
    }

    /**
     * Compares two swerve module states. One swerve module is "greater" than the
     * other if its speed is higher than the other.
     *
     * @param other The other swerve module.
     * @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
     */
    @Override
    public int compareTo(SwerveModuleState100 other) {
        return Double.compare(this.m_speedM_S, other.m_speedM_S);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof SwerveModuleState100) {
            SwerveModuleState100 other = (SwerveModuleState100) obj;
            return Math.abs(other.m_speedM_S - m_speedM_S) < 1E-9
                    && m_angle.equals(other.m_angle);
        }
        return false;
    }

    /** SwerveModuleState struct for serialization. */
    public static final SwerveModuleStateStruct struct = new SwerveModuleStateStruct();
}
