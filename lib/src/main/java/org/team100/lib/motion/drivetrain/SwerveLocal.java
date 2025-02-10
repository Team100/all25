package org.team100.lib.motion.drivetrain;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePositions;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.state.Control100;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The swerve drive in local, or robot, reference frame. This class knows
 * nothing about the outside world, it just accepts chassis speeds.
 */
public class SwerveLocal implements Glassy, SwerveLocalObserver {
    private static final double kPositionToleranceRad = 0.05; // about 3 degrees
    private static final SwerveModuleStates states0 = new SwerveModuleStates(
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
    private static final SwerveModuleStates states90 = new SwerveModuleStates(
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))));
    private static final SwerveModuleStates statesX = new SwerveModuleStates(
            // note range is [-pi,pi]
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(-1 * Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(3 * Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(-3 * Math.PI / 4))));

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final SwerveModuleCollection m_modules;
    private final AsymSwerveSetpointGenerator m_SwerveSetpointGenerator;
    private final ChassisSpeedsLogger m_log_desired;
    private final ChassisSpeedsLogger m_log_setpoint_delta;
    private final ChassisSpeedsLogger m_log_prev_setpoint;
    private final ChassisSpeedsLogger m_log_setpoint;
    private final ChassisSpeedsLogger m_log_chassis_speed;

    private SwerveSetpoint m_prevSetpoint;

    public SwerveLocal(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            AsymSwerveSetpointGenerator setpointGenerator,
            SwerveModuleCollection modules) {
        LoggerFactory child = parent.child(this);
        m_log_desired = child.chassisSpeedsLogger(Level.DEBUG, "desired chassis speed");
        m_log_setpoint_delta = child.chassisSpeedsLogger(Level.TRACE, "setpoint delta");
        m_log_prev_setpoint = child.chassisSpeedsLogger(Level.TRACE, "prevSetpoint chassis speed");
        m_log_setpoint = child.chassisSpeedsLogger(Level.DEBUG, "setpoint chassis speed");
        m_log_chassis_speed = child.chassisSpeedsLogger(Level.TRACE, "chassis speed LOG");
        m_swerveKinodynamics = swerveKinodynamics;
        m_modules = modules;
        m_SwerveSetpointGenerator = setpointGenerator;
        m_prevSetpoint = new SwerveSetpoint();
    }

    //////////////////////////////////////////////////////////
    //
    // Actuators. These are mutually exclusive within an iteration.
    //

    /**
     * Drives the modules to produce the target chassis speed.
     * 
     * Feasibility is enforced by the setpoint generator (if enabled) and the
     * desaturator.
     * 
     * @param speeds        speeds in robot coordinates.
     * @param gyroRateRad_S
     * @param kDtSec        time in the future for the setpoint generator to
     *                      calculate
     */
    public void setChassisSpeeds(ChassisSpeeds speeds, double gyroRateRad_S) {
        m_log_desired.log(() -> speeds);
        if (Experiments.instance.enabled(Experiment.UseSetpointGenerator)) {
            setChassisSpeedsWithSetpointGenerator(speeds);
        } else {
            setChassisSpeedsNormally(speeds, gyroRateRad_S);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setChassisSpeedsNormally(speeds, 0);
    }

    public void setChassisSpeedsNormally(ChassisSpeeds speeds, double gyroRateRad_S) {
        SwerveModuleStates states = m_swerveKinodynamics.toSwerveModuleStates(speeds, gyroRateRad_S);
        setModuleStates(states);
        m_prevSetpoint = new SwerveSetpoint(speeds, states);
        m_log_chassis_speed.log(() -> speeds);
    }

    /**
     * True if current wheel steering positions are aligned with the positions
     * implied by the desired speed.
     */
    public boolean aligned(ChassisSpeeds desiredSpeeds, double gyroRateRad_S) {
        // These measurements include empty angles for motionless wheels.
        // Otherwise angle is always within [-pi, pi].
        OptionalDouble[] measurements = m_modules.turningPosition();
        SwerveModuleState100[] setpoints = m_swerveKinodynamics.toSwerveModuleStates(
                desiredSpeeds, gyroRateRad_S).all();
        for (int i = 0; i < measurements.length; ++i) {
            if (measurements[i].isEmpty()) {
                // Util.warn("broken sensor, this should never happen");
                return false;
            }
            double measurement = measurements[i].getAsDouble();
            if (setpoints[i].angle().isEmpty()) {
                // This can happen if the desired wheel speed is zero, which can happen even if
                // the other desired wheel speeds are not zero.
                // Util.printf("angle %d empty\n", i);
                continue;
            }
            double setpoint = setpoints[i].angle().get().getRadians();
            double error = MathUtil.angleModulus(setpoint - measurement);
            if (Math.abs(error) > kPositionToleranceRad) {
                // Util.printf("angle %d error %f  outside tolerance\n", i, error);
                return false;
            }
        }
        // Util.println("all good");
        return true;
    }

    /**
     * @return true if aligned
     */
    public boolean steerAtRest(ChassisSpeeds speeds, double gyroRateRad_S) {
        // this indicates that during the steering the goal is fixed
        SwerveModuleStates states = m_swerveKinodynamics.toSwerveModuleStates(
                speeds, gyroRateRad_S);

        states = states.motionless();

        setModuleStates(states);
        // previous setpoint should be at rest with the current states
        m_prevSetpoint = new SwerveSetpoint(new ChassisSpeeds(), states);
        return Util.all(atGoal());
    }

    /**
     * Sets the wheels to make an "X" pattern.
     */
    public void defense() {
        // not optimizing makes it easier to test, not sure it's worth the slowness.
        setRawModuleStates(statesX);
    }

    /**
     * Sets wheel rotation to zero, for optimizing steering control.
     */
    public void steer0() {
        setRawModuleStates(states0);
    }

    /**
     * Sets wheel rotation to 90 degrees, for optimizing steering control.
     */
    public void steer90() {
        setRawModuleStates(states90);
    }

    public void stop() {
        m_modules.stop();
    }

    /**
     * Set the module states without desaturating.
     * 
     * Works fine with empty angles.
     * 
     * This "raw" mode is just for testing.
     */
    public void setRawModuleStates(SwerveModuleStates targetModuleStates) {
        m_modules.setRawDesiredStates(targetModuleStates);
    }

    ////////////////////////////////////////////////////////////////////
    //
    // Observers
    //

    @Override
    public SwerveModuleStates getDesiredStates() {
        return m_modules.getDesiredStates();
    }

    public Control100[] getSetpoints() {
        return m_modules.getSetpoint();
    }

    @Override
    public SwerveModuleStates states() {
        return m_modules.states();
    }

    @Override
    public SwerveModulePositions positions() {
        return m_modules.positions();
    }

    public Translation2d[] getModuleLocations() {
        return m_swerveKinodynamics.getKinematics().getModuleLocations();
    }

    public boolean[] atSetpoint() {
        return m_modules.atSetpoint();
    }

    @Override
    public boolean[] atGoal() {
        return m_modules.atGoal();
    }

    ///////////////////////////////////////////

    public void close() {
        m_modules.close();
    }

    public void reset() {
        Util.warn("make sure resetting in SwerveLocal doesn't break anything");
        m_modules.reset();
    }

    public void resetSetpoint(SwerveSetpoint setpoint) {
        m_prevSetpoint = setpoint;
    }

    /** Updates visualization. */
    void periodic() {
        m_modules.periodic();
    }

    /////////////////////////////////////////////////////////

    private void setChassisSpeedsWithSetpointGenerator(ChassisSpeeds speeds) {
        // Informs SwerveDriveKinematics of the module states.
        SwerveSetpoint setpoint = m_SwerveSetpointGenerator.generateSetpoint(
                m_prevSetpoint,
                speeds);
        // ideally delta would be zero because our input would be feasible.
        ChassisSpeeds delta = setpoint.getChassisSpeeds().minus(speeds);
        m_log_setpoint_delta.log(() -> delta);
        m_log_prev_setpoint.log(m_prevSetpoint::getChassisSpeeds);
        m_log_setpoint.log(setpoint::getChassisSpeeds);
        setModuleStates(setpoint.getModuleStates());
        m_prevSetpoint = setpoint;
    }

    /**
     * No longer desaturates. If you want desaturation, use the setpoint generator.
     * Works fine with empty angles.
     */
    private void setModuleStates(SwerveModuleStates states) {
        m_modules.setDesiredStates(states);
    }
}
