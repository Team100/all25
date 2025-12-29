package org.team100.lib.examples.motion;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.rev.CANSparkMotor;
import org.team100.lib.motor.rev.NeoCANSparkMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.Gravity;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Similar to RotaryPositionSubsystem1d but uses outboard positional control,
 * which is what we always do if we can.
 */
public class OutboardRotaryPositionSubsystem extends SubsystemBase {
    /** Home position, rad */
    private static final double HOME = 0.1;
    /** Extended position, rad */
    private static final double EXTEND = 2;

    private final Gravity m_gravity;
    private final AngularPositionServo m_servo;

    public OutboardRotaryPositionSubsystem(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        m_gravity = new Gravity(log,
                5, // Max gravity torque, Nm
                0); // Gravity torque position offset, rad
        IncrementalProfile profile = new TrapezoidIncrementalProfile(
                log,
                1, // max velocity rad/s
                1, // max accel rad/s^2
                0.01); // tolerance
        ProfileReferenceR1 ref = new IncrementalProfileReferenceR1(log, () -> profile,
                0.01, // position tolerance, rad
                0.01); // velocity tolerance, rad/s
        m_servo = new OutboardAngularPositionServo(log, mech(log), ref);
        m_servo.reset();
    }

    private RotaryMechanism mech(LoggerFactory log) {
        switch (Identity.instance) {
            case BLANK -> {
                // simulation
                SimulatedBareMotor motor = new SimulatedBareMotor(log, 600);
                IncrementalBareEncoder encoder = motor.encoder();
                return getMech(log, motor, encoder);
            }
            default -> {
                // real robot
                CANSparkMotor motor = new NeoCANSparkMotor(
                        log,
                        new CanId(0),
                        NeutralMode.BRAKE,
                        MotorPhase.FORWARD,
                        10, // Stator current limit, amps
                        Feedforward100.makeNeo(log),
                        PIDConstants.makePositionPID(log, 0.0003) // duty-cycle/RPM
                );
                IncrementalBareEncoder encoder = motor.encoder();
                return getMech(log, motor, encoder);
            }
        }

    }

    private RotaryMechanism getMech(LoggerFactory log, BareMotor motor, IncrementalBareEncoder encoder) {
        RotaryMechanism mech = new RotaryMechanism(log, motor,
                encoder,
                0, // initial position, rad
                25, // gear ratio
                0, // min position, rad
                3); // max position, rad
        return mech;
    }

    public double getWrappedPositionRad() {
        return m_servo.getWrappedPositionRad();
    }

    /**
     * Go home (with gravity) and then hold position against the hard stop (without
     * gravity) forever.
     */
    public Command home() {
        Command goHome = run(() -> m_servo.setPositionProfiled(HOME, gravityTorque()));
        Command stayHome = run(() -> m_servo.setPositionProfiled(HOME, 0));
        return goHome.until(m_servo::atGoal).andThen(stayHome);
    }

    /** Go to the extended position, and hold there forever. */
    public Command extend() {
        return run(() -> m_servo.setPositionProfiled(EXTEND, gravityTorque()));
    }

    /////////////////////////////////////////////////////////////////

    private double gravityTorque() {
        return m_gravity.applyAsDouble(m_servo.getWrappedPositionRad());
    }
}
