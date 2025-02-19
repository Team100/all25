package org.team100.lib.motor;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.util.Memo;
import org.team100.lib.util.Util;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotController;

public abstract class CANSparkMotor implements BareMotor {
    protected final Feedforward100 m_ff;
    protected final SparkBase m_motor;
    protected final RelativeEncoder m_encoder;
    protected final SparkClosedLoopController m_pidController;
    // CACHES
    private final DoubleSupplier m_encoder_position;
    private final DoubleSupplier m_encoder_velocity;
    private final DoubleSupplier m_current;
    private final DoubleSupplier m_supplyVoltage;
    private final DoubleSupplier m_output;
    private final DoubleSupplier m_temp;
    // LOGGERS
    private final DoubleLogger m_log_desired_position;
    private final DoubleLogger m_log_desired_speed;
    private final DoubleLogger m_log_desired_accel;
    private final DoubleLogger m_log_friction_FF;
    private final DoubleLogger m_log_velocity_FF;
    private final DoubleLogger m_log_accel_FF;
    private final DoubleLogger m_log_torque_FF;
    private final DoubleLogger m_log_duty;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_rpm;
    private final DoubleLogger m_log_current;
    private final DoubleLogger m_log_supplyVoltage;
    private final DoubleLogger m_log_torque;
    private final DoubleLogger m_log_temp;

    protected CANSparkMotor(
            LoggerFactory parent,
            SparkBase motor,
            MotorPhase motorPhase,
            int currentLimit,
            Feedforward100 ff,
            PIDConstants pid) {
        m_motor = motor;
        LoggerFactory child = parent.child(this);
        m_ff = ff;
        // make config synchronous so we can see the errors
        Rev100.crash(() -> m_motor.setCANTimeout(500));
        Rev100.baseConfig(m_motor);
        Rev100.motorConfig(m_motor, IdleMode.kCoast, motorPhase, 20);
        Rev100.currentConfig(m_motor, currentLimit);
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getClosedLoopController();
        Rev100.pidConfig(m_motor, pid);
        // make everything after this asynchronous.
        // NOTE: this makes error-checking not work at all.
        Rev100.crash(() -> m_motor.setCANTimeout(0));
        // CACHES
        m_encoder_position = Memo.ofDouble(m_encoder::getPosition);
        m_encoder_velocity = Memo.ofDouble(m_encoder::getVelocity);
        m_current = Memo.ofDouble(m_motor::getOutputCurrent);
        m_supplyVoltage = Memo.ofDouble(m_motor::getBusVoltage);
        m_output = Memo.ofDouble(m_motor::getAppliedOutput);
        m_temp = Memo.ofDouble(m_motor::getMotorTemperature);
        // LOGGERS
        child.intLogger(Level.TRACE, "Device ID").log(m_motor::getDeviceId);
        m_log_desired_position = child.doubleLogger(Level.DEBUG, "desired position (rev)");
        m_log_desired_speed = child.doubleLogger(Level.DEBUG, "desired speed (rev_s)");
        m_log_desired_accel = child.doubleLogger(Level.TRACE, "desired accel (rev_s2)");
        m_log_friction_FF = child.doubleLogger(Level.TRACE, "friction feedforward (v)");
        m_log_velocity_FF = child.doubleLogger(Level.TRACE, "velocity feedforward (v)");
        m_log_accel_FF = child.doubleLogger(Level.TRACE, "accel feedforward (v)");
        m_log_torque_FF = child.doubleLogger(Level.TRACE, "torque feedforward (v)");
        m_log_duty = child.doubleLogger(Level.DEBUG, "Duty Cycle");
        m_log_position = child.doubleLogger(Level.DEBUG, "position (rev)");
        m_log_velocity = child.doubleLogger(Level.DEBUG, "velocity (rev_s)");
        m_log_rpm = child.doubleLogger(Level.TRACE, "velocity (RPM)");
        m_log_current = child.doubleLogger(Level.DEBUG, "current (A)");
        m_log_supplyVoltage = child.doubleLogger(Level.DEBUG, "voltage (V)");
        m_log_torque = child.doubleLogger(Level.TRACE, "torque (Nm)");
        m_log_temp = child.doubleLogger(Level.TRACE, "temperature (C)");
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        m_log_duty.log(() -> output);
        log();
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        int currentA = (int) (torqueNm / kTNm_amp());
        Rev100.currentConfig(m_motor, currentA);
    }

    /**
     * Use outboard PID control to hold the given velocity, with velocity,
     * acceleration, and torque feedforwards.
     */
    @Override
    public void setVelocity(double motorRad_S, double motorAccelRad_S2, double motorTorqueNm) {
        final double motorRev_S = motorRad_S / (2 * Math.PI);
        final double motorRev_S2 = motorAccelRad_S2 / (2 * Math.PI);
        final double currentMotorRev_S = m_encoder_velocity.getAsDouble() / 60;

        final double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        final double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        final double accelFFVolts = m_ff.accelFFVolts(motorRev_S2);
        final double torqueFFVolts = getTorqueFFVolts(motorTorqueNm);

        final double kFF = frictionFFVolts + velocityFFVolts + accelFFVolts + torqueFFVolts;

        final double motorRev_M = motorRev_S * 60;
        Rev100.warn(() -> m_pidController.setReference(
                motorRev_M, ControlType.kVelocity, ClosedLoopSlot.kSlot1, kFF, ArbFFUnits.kVoltage));

        m_log_desired_speed.log(() -> motorRev_S);
        m_log_desired_accel.log(() -> motorRev_S2);
        m_log_friction_FF.log(() -> frictionFFVolts);
        m_log_velocity_FF.log(() -> velocityFFVolts);
        m_log_accel_FF.log(() -> accelFFVolts);
        m_log_torque_FF.log(() -> torqueFFVolts);
        log();
    }

    /**
     * Use outboard PID control to hold the given position, with velocity and torque
     * feedforwards.
     * 
     * Motor revolutions wind up, so setting 0 revs and 1 rev are different.
     */
    @Override
    public void setPosition(double motorPositionRad, double motorVelocityRad_S, double motorAccelRad_S2, double motorTorqueNm) {
        final double motorRev = motorPositionRad / (2 * Math.PI);
        final double motorRev_S = motorVelocityRad_S / (2 * Math.PI);
        final double currentMotorRev_S = m_encoder_velocity.getAsDouble() / 60;

        final double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        final double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        final double torqueFFVolts = getTorqueFFVolts(motorTorqueNm);

        final double kFF = frictionFFVolts + velocityFFVolts + torqueFFVolts;

        Rev100.warn(() -> m_pidController.setReference(
                motorRev, ControlType.kPosition, ClosedLoopSlot.kSlot0, kFF, ArbFFUnits.kVoltage));

        m_log_desired_position.log(() -> motorRev);
        m_log_desired_speed.log(() -> motorRev_S);
        m_log_friction_FF.log(() -> frictionFFVolts);
        m_log_velocity_FF.log(() -> velocityFFVolts);
        m_log_torque_FF.log(() -> torqueFFVolts);
        log();
    }

    /** Value is updated in Robot.robotPeriodic(). */
    @Override
    public double getVelocityRad_S() {
        return getRateRPM() * 2 * Math.PI / 60;
    }

    @Override
    public void setEncoderPositionRad(double positionRad) {
        setEncoderPosition(positionRad / (2 * Math.PI));
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    /**
     * Value is updated in Robot.robotPeriodic().
     * 
     * @return torque in Nm
     */
    public double getMotorTorque() {
        return m_current.getAsDouble() * kTNm_amp();
    }

    /**
     * Value is updated in Robot.robotPeriodic().
     * 
     * @return integrated sensor position in rotations.
     */
    public double getPositionRot() {
        return m_encoder_position.getAsDouble();
    }

    /**
     * Value is updated in Robot.robotPeriodic().
     * 
     * @return integrated sensor velocity in RPM
     */
    public double getRateRPM() {
        return m_encoder_velocity.getAsDouble();
    }

    /**
     * Sets integrated sensor position to zero.
     */
    public void resetEncoderPosition() {
        Rev100.warn(() -> m_encoder.setPosition(0));
    }

    /**
     * Set integrated sensor position in rotations.
     */
    public void setEncoderPosition(double motorPositionRev) {
        Rev100.warn(() -> m_encoder.setPosition(motorPositionRev));
    }

    protected void log() {
        m_log_position.log(m_encoder_position);
        m_log_velocity.log(() -> m_encoder_velocity.getAsDouble() / 60);
        m_log_rpm.log(m_encoder_velocity);
        m_log_current.log(m_current);
        m_log_supplyVoltage.log(m_supplyVoltage);
        m_log_duty.log(m_output);
        m_log_torque.log(this::getMotorTorque);
        m_log_temp.log(m_temp);
        if (RobotController.getBatteryVoltage() - m_supplyVoltage.getAsDouble() > 1)
            Util.warnf("Motor voltage %d low, bad connection? motor: %f, battery: %f\n",
                    m_motor.getDeviceId(),
                    m_supplyVoltage.getAsDouble(),
                    RobotController.getBatteryVoltage());
    }

    @Override
    public void periodic() {
        log();
    }
}
