package org.team100.lib.motor;

import java.util.function.Supplier;

import org.team100.lib.config.PIDConstants;
import org.team100.lib.util.Util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Rev100 {

    public static void crash(Supplier<REVLibError> s) {
        REVLibError errorCode = s.get();
        if (errorCode != REVLibError.kOk) {
            throw new IllegalStateException(errorCode.name());
        }
    }

    public static void warn(Supplier<REVLibError> s) {
        REVLibError errorCode = s.get();
        if (errorCode != REVLibError.kOk) {
            Util.warn(errorCode.name());
        }
    }

    public static void baseConfig(SparkBase motor) {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.limitSwitch.forwardLimitSwitchEnabled(false);
        conf.limitSwitch.reverseLimitSwitchEnabled(false);
        conf.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        conf.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        crash(() -> motor.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    public static void motorConfig(SparkBase motor, IdleMode idleMode, MotorPhase phase,
            int periodMs) {
        setIdleMode(motor, idleMode);
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.inverted(phase == MotorPhase.REVERSE);
        conf.signals.primaryEncoderVelocityPeriodMs(periodMs);
        conf.signals.primaryEncoderVelocityAlwaysOn(true);
        conf.signals.primaryEncoderPositionPeriodMs(periodMs);
        conf.signals.primaryEncoderPositionAlwaysOn(true);
        // slower than default of 10; also affects things like motor temperature and applied output.
        conf.signals.limitsPeriodMs(20);
        crash(() -> motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    public static void currentConfig(SparkBase motor, int currentLimit) {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.smartCurrentLimit(currentLimit);
        crash(() -> motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    public static void setIdleMode(SparkBase motor, IdleMode idleMode) {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.idleMode(idleMode);
        crash(() -> motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    /**
     * The PID parameters here use units of duty cycle per RPM, so for a typical
     * motor speed of a few thousand RPM, and a typical error of a few hundred RPM,
     * a desired control output would be a duty cycle of 0.1 or so, which implies a
     * value of P like 3e-4.
     */
    public static void pidConfig(SparkBase motor, PIDConstants pid) {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.closedLoop.positionWrappingEnabled(false); // don't use position control
        conf.closedLoop.p(pid.getPositionP(), ClosedLoopSlot.kSlot0);
        conf.closedLoop.i(pid.getPositionI(), ClosedLoopSlot.kSlot0);
        conf.closedLoop.d(pid.getPositionD(), ClosedLoopSlot.kSlot0);
        conf.closedLoop.p(pid.getVelocityP(), ClosedLoopSlot.kSlot1);
        conf.closedLoop.i(pid.getVelocityP(), ClosedLoopSlot.kSlot1);
        conf.closedLoop.d(pid.getVelocityP(), ClosedLoopSlot.kSlot1);
        conf.closedLoop.iZone(pid.getPositionIZone(), ClosedLoopSlot.kSlot0);
        conf.closedLoop.iZone(pid.getVelocityIZone(), ClosedLoopSlot.kSlot1);
        conf.closedLoop.velocityFF(0); // use arbitrary FF instead
        conf.closedLoop.outputRange(-1, 1);
        crash(() -> motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    private Rev100() {
        //
    }

}
