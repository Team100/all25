package org.team100.lib.subsystems.five_bar;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.ctre.Falcon6Motor;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.subsystems.five_bar.kinematics.FiveBarKinematics;
import org.team100.lib.subsystems.five_bar.kinematics.JointPositions;
import org.team100.lib.subsystems.five_bar.kinematics.Scenario;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Control at the "servo" level, which includes a profile. For multi-axis
 * mechanisms this is not the best approach, because the profiles are not
 * coordinated.
 */
public class FiveBarServo extends SubsystemBase {
    private static final double POSITION_TOLERANCE = 0.05;
    private static final double VELOCITY_TOLERANCE = 0.05;
    /** Low current limits */
    private static final double SUPPLY_LIMIT = 5;
    private static final double STATOR_LIMIT = 5;
    private static final double MAX_VELOCITY = 190;
    private static final double MAX_ACCEL = 210;
    private static final Scenario SCENARIO;
    static {
        // origin is P1
        SCENARIO = new Scenario();
        // TODO: real measurements
        SCENARIO.a1 = 0.1;
        SCENARIO.a2 = 0.1;
        SCENARIO.a3 = 0.1;
        SCENARIO.a4 = 0.1;
        SCENARIO.a5 = 0.1;
        SCENARIO.xcenter = 0.5;
        SCENARIO.ycenter = 0.15;
    }

    /** Left motor, "P1" in the diagram. */
    /**
     * There's no absolute encoder in the apparatus, so we use a "proxy" instead;
     * this needs a "homing" mechanism of some kind.
     */
    private final ProxyRotaryPositionSensor m_sensorP1;
    private final AngularPositionServo m_servoP1;

    /** Right motor, "P5" in the diagram. */
    private final ProxyRotaryPositionSensor m_sensorP5;
    private final AngularPositionServo m_servoP5;

    public FiveBarServo(LoggerFactory logger) {
        // zeros
        PIDConstants pid = PIDConstants.zero(logger);
        Feedforward100 ff = Feedforward100.zero(logger);
        IncrementalProfile profile = new TrapezoidIncrementalProfile(
                logger, MAX_VELOCITY, MAX_ACCEL, POSITION_TOLERANCE);

        LoggerFactory loggerP1 = logger.name("p1");
        Falcon6Motor motorP1 = new Falcon6Motor(
                loggerP1,
                new CanId(1),
                NeutralMode.COAST,
                MotorPhase.FORWARD,
                SUPPLY_LIMIT,
                STATOR_LIMIT,
                pid,
                ff);
        IncrementalBareEncoder encoderP1 = motorP1.encoder();
        m_sensorP1 = new ProxyRotaryPositionSensor(encoderP1, 1.0);
        RotaryMechanism mechP1 = new RotaryMechanism(
                loggerP1,
                motorP1,
                m_sensorP1,
                1.0,
                0.0,
                1.0);

        ProfileReferenceR1 refP1 = new IncrementalProfileReferenceR1(
                loggerP1, () -> profile, POSITION_TOLERANCE, VELOCITY_TOLERANCE);
        m_servoP1 = new OutboardAngularPositionServo(
                loggerP1,
                mechP1,
                refP1);

        LoggerFactory loggerP5 = logger.name("p5");
        Falcon6Motor motorP5 = new Falcon6Motor(
                loggerP5,
                new CanId(2),
                NeutralMode.COAST,
                MotorPhase.FORWARD,
                SUPPLY_LIMIT,
                STATOR_LIMIT,
                pid,
                ff);
        IncrementalBareEncoder encoderP5 = motorP5.encoder();
        m_sensorP5 = new ProxyRotaryPositionSensor(encoderP5, 1.0);
        RotaryMechanism m_mechP5 = new RotaryMechanism(
                loggerP5,
                motorP5,
                m_sensorP5,
                1.0,
                0.0,
                1.0);
        ProfileReferenceR1 refP5 = new IncrementalProfileReferenceR1(
                loggerP5, () -> profile, POSITION_TOLERANCE, VELOCITY_TOLERANCE);
        m_servoP5 = new OutboardAngularPositionServo(
                loggerP5,
                m_mechP5,
                refP5);
    }

    public void setPosition(double p1, double p5) {
        m_servoP1.setPositionProfiled(p1, 0);
        m_servoP5.setPositionProfiled(p5, 0);
    }

    public JointPositions getJointPositions() {
        double q1 = m_servoP1.getWrappedPositionRad();
        double q5 = m_servoP5.getWrappedPositionRad();
        return FiveBarKinematics.forward(SCENARIO, q1, q5);
    }

    @Override
    public void periodic() {
        m_servoP1.periodic();
        m_servoP5.periodic();
    }

    //////////////////////

    private void setDutyCycle(double p1, double p5) {
        m_servoP1.setDutyCycle(p1);
        m_servoP5.setDutyCycle(p5);
    }

    private void resetEncoderPosition() {
        m_sensorP1.setEncoderPosition(0);
        m_sensorP5.setEncoderPosition(0);
    }

    ///////////////////////
    //
    // Commands

    public Command home() {
        return run(() -> setDutyCycle(0.05, 0.05));
    }

    public Command zero() {
        return runOnce(this::resetEncoderPosition);
    }

    public Command position(DoubleSupplier p1, DoubleSupplier p5) {
        return run(() -> setPosition(p1.getAsDouble(), p5.getAsDouble()));
    }
}
