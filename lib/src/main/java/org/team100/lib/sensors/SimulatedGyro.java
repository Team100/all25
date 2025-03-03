package org.team100.lib.sensors;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.util.Takt;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A simulated gyro that uses drivetrain odometry.
 */
public class SimulatedGyro implements Gyro {
    private double m_heading = 0;
    private final SwerveKinodynamics m_kinodynamics;
    private final SwerveModuleCollection m_moduleCollection;
    private double m_time = Takt.get();

    public SimulatedGyro(
            SwerveKinodynamics kinodynamics,
            SwerveModuleCollection collection) {
        m_kinodynamics = kinodynamics;
        m_moduleCollection = collection;
    }

    @Override
    public Rotation2d getYawNWU() {
        SwerveModuleStates states = m_moduleCollection.states();
        ChassisSpeeds speeds = m_kinodynamics.toChassisSpeedsWithDiscretization(states, 0.02);
        double now = Takt.get();
        double dt = now - m_time;
        m_heading += speeds.omegaRadiansPerSecond * dt;
        m_time = now;
        return new Rotation2d(m_heading);
    }

    @Override
    public double getYawRateNWU() {
        SwerveModuleStates states = m_moduleCollection.states();
        ChassisSpeeds speeds = m_kinodynamics.toChassisSpeedsWithDiscretization(states, 0.02);
        return speeds.omegaRadiansPerSecond;
    }

    @Override
    public Rotation2d getPitchNWU() {
        return Rotation2d.kZero;
    }

    @Override
    public Rotation2d getRollNWU() {
        return Rotation2d.kZero;
    }

    @Override
    public void periodic() {
        //
    }

}
