package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.util.Memo;
import org.team100.lib.util.Memo.DoubleCache;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * Superclass for TalonFX motors.
 */
public abstract class Talon6Motor implements BareMotor {
    // speeding up the updates is a tradeoff between latency and CAN utilization.
    // 254 seems to think that 100 is a good compromise?
    // see
    // https://github.com/Team254/FRC-2024-Public/blob/040f653744c9b18182be5f6bc51a7e505e346e59/src/main/java/com/team254/lib/ctre/swerve/SwerveDrivetrain.java#L382
    private static final int SIGNAL_UPDATE_FREQ_HZ = 100;
    private final TalonFX m_motor;
    private final Feedforward100 m_ff;

    // CACHES
    // Two levels of caching here: the cotemporal cache caches the value
    // and also the supplier
    /** position is latency-compensated. */
    protected final DoubleCache m_position;
    protected final DoubleCache m_velocity;
    protected final DoubleCache m_acceleration;
    protected final DoubleCache m_dutyCycle;
    protected final DoubleCache m_error;
    protected final DoubleCache m_supply;
    protected final DoubleCache m_supplyVoltage;
    protected final DoubleCache m_stator;
    protected final DoubleCache m_temp;
    protected final DoubleCache m_torque;

    // caching the control requests saves allocation
    private final VelocityVoltage m_velocityVoltage;
    private final DutyCycleOut m_dutyCycleOut;
    private final PositionVoltage m_positionVoltage;

    private final double m_supplyLimit;

    // LOGGERS
    private final DoubleLogger m_log_desired_duty;
    private final DoubleLogger m_log_desired_position;
    private final DoubleLogger m_log_desired_speed;
    private final DoubleLogger m_log_desired_accel;
    private final DoubleLogger m_log_friction_FF;
    private final DoubleLogger m_log_velocity_FF;
    private final DoubleLogger m_log_accel_FF;
    private final DoubleLogger m_totalFeedForward;
    private final DoubleLogger m_log_torque_FF;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_accel;
    private final DoubleLogger m_log_output;
    private final DoubleLogger m_log_error;
    private final DoubleLogger m_log_supply;
    private final DoubleLogger m_log_supplyVoltage;
    private final DoubleLogger m_log_stator;
    private final DoubleLogger m_log_torque;
    private final DoubleLogger m_log_temp;

    protected Talon6Motor(
            LoggerFactory parent,
            int canId,
            MotorPhase motorPhase,
            double supplyLimit,
            double statorLimit,
            PIDConstants lowLevelPIDConstants,
            Feedforward100 ff) {
        m_velocityVoltage = new VelocityVoltage(0);
        m_dutyCycleOut = new DutyCycleOut(0);
        m_positionVoltage = new PositionVoltage(0);
        // make control synchronous, i.e. "actuate immediately."
        // see
        // https://github.com/Team254/FRC-2024-Public/blob/040f653744c9b18182be5f6bc51a7e505e346e59/src/main/java/com/team254/lib/ctre/swerve/SwerveModule.java#L210
        m_velocityVoltage.UpdateFreqHz = 0;
        m_dutyCycleOut.UpdateFreqHz = 0;
        m_positionVoltage.UpdateFreqHz = 0;

        LoggerFactory child = parent.child(this);
        m_motor = new TalonFX(canId);
        m_ff = ff;
        m_supplyLimit = supplyLimit;

        TalonFXConfigurator talonFXConfigurator = m_motor.getConfigurator();
        Phoenix100.logCrashStatus();
        Phoenix100.baseConfig(talonFXConfigurator);
        Phoenix100.motorConfig(talonFXConfigurator, motorPhase);
        Phoenix100.currentConfig(talonFXConfigurator, supplyLimit, statorLimit);
        Phoenix100.pidConfig(talonFXConfigurator, lowLevelPIDConstants);

        Phoenix100.crash(() -> m_motor.getPosition().setUpdateFrequency(SIGNAL_UPDATE_FREQ_HZ));
        Phoenix100.crash(() -> m_motor.getVelocity().setUpdateFrequency(SIGNAL_UPDATE_FREQ_HZ));
        Phoenix100.crash(() -> m_motor.getAcceleration().setUpdateFrequency(SIGNAL_UPDATE_FREQ_HZ));
        Phoenix100.crash(() -> m_motor.getTorqueCurrent().setUpdateFrequency(SIGNAL_UPDATE_FREQ_HZ));

        // Cache the status signal getters.
        final StatusSignal<Angle> motorPosition = m_motor.getPosition();
        final StatusSignal<AngularVelocity> motorVelocity = m_motor.getVelocity();
        final StatusSignal<AngularAcceleration> motorAcceleration = m_motor.getAcceleration();
        final StatusSignal<Double> motorDutyCycle = m_motor.getDutyCycle();
        final StatusSignal<Double> motorClosedLoopError = m_motor.getClosedLoopError();
        final StatusSignal<Current> motorSupplyCurrent = m_motor.getSupplyCurrent();
        final StatusSignal<Voltage> motorSupplyVoltage = m_motor.getSupplyVoltage();
        final StatusSignal<Current> motorStatorCurrent = m_motor.getStatorCurrent();
        final StatusSignal<Temperature> motorDeviceTemp = m_motor.getDeviceTemp();
        final StatusSignal<Current> motorTorqueCurrent = m_motor.getTorqueCurrent();

        // The memoizer refreshes all the signals at once.
        Memo.registerSignal(motorPosition);
        Memo.registerSignal(motorVelocity);
        Memo.registerSignal(motorAcceleration);
        Memo.registerSignal(motorDutyCycle);
        Memo.registerSignal(motorClosedLoopError);
        Memo.registerSignal(motorSupplyCurrent);
        Memo.registerSignal(motorSupplyVoltage);
        Memo.registerSignal(motorStatorCurrent);
        Memo.registerSignal(motorDeviceTemp);
        Memo.registerSignal(motorTorqueCurrent);

        // None of these need to refresh.
        // this latency compensation uses takt time rather than the real clock.
        m_position = Memo
                .ofDouble(() -> {
                    double latency = Utils.fpgaToCurrentTime(Takt.get()) - motorPosition.getTimestamp().getTime();
                    if (latency > 0.1) {
                        Util.warn("!!!!!!! stale position! !!!!!!!");
                        latency = 0.1;
                    }
                    return motorPosition.getValueAsDouble() + (motorVelocity.getValueAsDouble() * latency);
                });
        m_velocity = Memo.ofDouble(() -> motorVelocity.getValueAsDouble());
        m_dutyCycle = Memo.ofDouble(() -> motorDutyCycle.getValueAsDouble());
        m_acceleration = Memo.ofDouble(() -> motorAcceleration.getValueAsDouble());
        m_error = Memo.ofDouble(() -> motorClosedLoopError.getValueAsDouble());
        m_supply = Memo.ofDouble(() -> motorSupplyCurrent.getValueAsDouble());
        m_supplyVoltage = Memo.ofDouble(() -> motorSupplyVoltage.getValueAsDouble());
        m_stator = Memo.ofDouble(() -> motorStatorCurrent.getValueAsDouble());
        m_temp = Memo.ofDouble(() -> motorDeviceTemp.getValueAsDouble());
        m_torque = Memo.ofDouble(() -> motorTorqueCurrent.getValueAsDouble());

        m_log_desired_duty = child.doubleLogger(Level.TRACE, "desired duty cycle [-1,1]");
        m_log_desired_position = child.doubleLogger(Level.DEBUG, "desired position (rev)");
        m_log_desired_speed = child.doubleLogger(Level.DEBUG, "desired speed (rev_s)");
        m_log_desired_accel = child.doubleLogger(Level.TRACE, "desired accel (rev_s2)");
        m_log_friction_FF = child.doubleLogger(Level.TRACE, "friction feedforward (v)");
        m_log_velocity_FF = child.doubleLogger(Level.TRACE, "velocity feedforward (v)");
        m_log_accel_FF = child.doubleLogger(Level.TRACE, "accel feedforward (v)");
        m_log_torque_FF = child.doubleLogger(Level.TRACE, "torque feedforward (v)");
        m_totalFeedForward = child.doubleLogger(Level.TRACE, "total feedforward (v)");

        m_log_position = child.doubleLogger(Level.DEBUG, "position (rev)");
        m_log_velocity = child.doubleLogger(Level.DEBUG, "velocity (rev_s)");
        m_log_accel = child.doubleLogger(Level.TRACE, "accel (rev_s2)");
        m_log_output = child.doubleLogger(Level.DEBUG, "output [-1,1]");
        m_log_error = child.doubleLogger(Level.TRACE, "error (rev_s)");
        m_log_supply = child.doubleLogger(Level.DEBUG, "supply current (A)");
        m_log_supplyVoltage = child.doubleLogger(Level.DEBUG, "supply voltage (V)");
        m_log_stator = child.doubleLogger(Level.TRACE, "stator current (A)");
        m_log_torque = child.doubleLogger(Level.TRACE, "torque (Nm)");
        m_log_temp = child.doubleLogger(Level.TRACE, "temperature (C)");

        child.intLogger(Level.TRACE, "Device ID").log(() -> canId);
    }

    /** Set duty cycle immediately. */
    @Override
    public void setDutyCycle(double output) {
        Phoenix100.warn(() -> m_motor.setControl(m_dutyCycleOut
                .withOutput(output)));
        m_log_desired_duty.log(() -> output);
        log();
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        int currentA = (int) (torqueNm / kTNm_amp());
        TalonFXConfigurator talonFXConfigurator = m_motor.getConfigurator();
        Phoenix100.currentConfig(talonFXConfigurator, m_supplyLimit, currentA);
    }

    /**
     * Use VelocityVoltage outboard PID control to hold the given velocity, with
     * friction, velocity, acceleration, and torque feedforwards.
     * 
     * Actuates immediately.
     */
    @Override
    public void setVelocity(double motorRad_S, double motorAccelRad_S2, double motorTorqueNm) {
        final double motorRev_S = motorRad_S / (2 * Math.PI);
        final double motorRev_S2 = motorAccelRad_S2 / (2 * Math.PI);
        final double currentMotorRev_S = m_velocity.getAsDouble();

        final double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        final double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        final double accelFFVolts = m_ff.accelFFVolts(currentMotorRev_S, motorRev_S2);
        final double torqueFFVolts = getTorqueFFVolts(motorTorqueNm);

        final double kFFVolts = frictionFFVolts + velocityFFVolts + accelFFVolts + torqueFFVolts;
        // final double kFFVolts = torqueFFVolts;

        // VelocityVoltage has an acceleration field for kA feedforward but we use
        // arbitrary feedforward for that.
        Phoenix100.warn(() -> m_motor.setControl(
                m_velocityVoltage
                        .withSlot(1)
                        .withVelocity(motorRev_S)
                        .withFeedForward(kFFVolts)));

        // without feedforward
        // Phoenix100.warn(() -> m_motor.setControl(
        // m_velocityVoltage
        // .withVelocity(motorRev_S)));

        m_log_desired_speed.log(() -> motorRev_S);
        m_log_desired_accel.log(() -> motorRev_S2);
        m_log_friction_FF.log(() -> frictionFFVolts);
        m_log_velocity_FF.log(() -> velocityFFVolts);
        m_log_accel_FF.log(() -> accelFFVolts);
        m_log_torque_FF.log(() -> torqueFFVolts);
        m_totalFeedForward.log(() -> kFFVolts);

        log();
    }

    /**
     * Use PositionVoltage outboard PID control to hold the given position, with
     * friction, velocity, and torque feedforwards.
     * 
     * Actuates immediately.
     * 
     * Motor revolutions wind up, so setting 0 revs and 1 rev are different.
     */
    @Override
    public void setPosition(double motorPositionRad, double motorVelocityRad_S, double motorAccelRad_S2, double motorTorqueNm) {
        final double motorRev = motorPositionRad / (2 * Math.PI);
        final double motorRev_S = motorVelocityRad_S / (2 * Math.PI);
        final double motorRev_S2 = motorAccelRad_S2 / (2 * Math.PI);

        final double currentMotorRev_S = m_velocity.getAsDouble();

        final double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        final double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        final double accelFFVolts = m_ff.accelFFVolts(currentMotorRev_S, motorRev_S2);
        final double torqueFFVolts = getTorqueFFVolts(motorTorqueNm);

        final double kFFVolts = frictionFFVolts + velocityFFVolts + torqueFFVolts + accelFFVolts;
        // final double kFFVolts = torqueFFVolts;

        // PositionVoltage has a velocity field for kV feedforward but we use arbitrary
        // feedforward for that.
        Phoenix100.warn(() -> m_motor.setControl(
                m_positionVoltage
                        .withSlot(0)
                        .withPosition(motorRev)
                        .withFeedForward(kFFVolts)));

        m_log_desired_position.log(() -> motorRev);
        m_log_desired_speed.log(() -> motorRev_S);
        m_log_desired_accel.log(() -> motorRev_S2);
        m_log_friction_FF.log(() -> frictionFFVolts);
        m_log_velocity_FF.log(() -> velocityFFVolts);
        m_log_torque_FF.log(() -> torqueFFVolts);
        m_log_accel_FF.log(() -> accelFFVolts);
        m_totalFeedForward.log(() -> kFFVolts);


        log();
    }

    /** Value is updated in Robot.robotPeriodic(). */
    @Override
    public double getVelocityRad_S() {
        return getVelocityRev_S() * 2 * Math.PI;
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
     * Sets integrated sensor position to zero.
     * 
     * Note this takes **FOREVER**, like tens of milliseconds, so you can only do it
     * at startup.
     */
    public void resetEncoderPosition() {
        Util.warn("Setting CTRE encoder position is very slow!");
        Phoenix100.warn(() -> m_motor.setPosition(0, 1));
        m_position.reset();
        m_velocity.reset();
    }

    /**
     * Set integrated sensor position in rotations.
     * 
     * Note this takes **FOREVER**, like tens of milliseconds, so you can only do it
     * at startup.
     */
    private void setEncoderPosition(double motorPositionRev) {
        Util.warn("Setting CTRE encoder position is very slow!");
        Phoenix100.warn(() -> m_motor.setPosition(motorPositionRev, 1));
    }

    /**
     * Set integrated sensor position in radians.
     * 
     * Note this takes **FOREVER**, like tens of milliseconds, so you can only do it
     * at startup.
     */
    @Override
    public void setEncoderPositionRad(double positionRad) {
        double motorPositionRev = positionRad / (2.0 * Math.PI);
        setEncoderPosition(motorPositionRev);
    }

    /** Updated in Robot.robotPeriodic(). */
    public double getVelocityRev_S() {
        return m_velocity.getAsDouble();
    }

    /** Updated in Robot.robotPeriodic(). */
    public double getPositionRev() {
        return m_position.getAsDouble();
    }

    /** wait a long time for a new value, do not use outside testing. */
    public double getPositionBlockingRev() {
        return m_motor.getPosition().waitForUpdate(1).getValueAsDouble();
    }

    protected void log() {
        m_log_position.log(m_position);
        m_log_velocity.log(m_velocity);
        m_log_accel.log(m_acceleration);
        m_log_output.log(m_dutyCycle);
        m_log_error.log(m_error);
        m_log_supply.log(m_supply);
        m_log_supplyVoltage.log(m_supplyVoltage);
        m_log_stator.log(m_stator);
        m_log_torque.log(this::getMotorTorque);
        m_log_temp.log(m_temp);
        // if (RobotController.getBatteryVoltage() - m_supplyVoltage.getAsDouble() > 1)
        // Util.warnf("Motor voltage %d low, bad connection? motor: %f, battery: %f\n",
        // m_motor.getDeviceID(),
        // m_supplyVoltage.getAsDouble(),
        // RobotController.getBatteryVoltage());
    }

    private double getMotorTorque() {
        // I looked into latency compensation of this signal but it doesn't seem
        // possible. latency compensation requires a signal and its time derivative,
        // e.g. position and velocity, or yaw and angular velocity. There doesn't seem
        // to be such a thing for current.
        return m_torque.getAsDouble() * kTNm_amp();
    }

    @Override
    public void periodic() {
        log();
    }
}
