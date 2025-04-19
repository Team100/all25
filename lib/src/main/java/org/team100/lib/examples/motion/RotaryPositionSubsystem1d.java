package org.team100.lib.examples.motion;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.FullStateFeedback;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.timed.JerkLimitedProfile100;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.reference.TimedProfileReference1d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Demonstrates how to assemble a one-dimensional subsystem with positional
 * control.
 * 
 * Examples of this sort of thing might be:
 * 
 * * a single-jointed arm
 * * an elevator
 * * the angle of a shooter
 * 
 * This class extends SubsystemBase, to be compatible with the scheduler, and it
 * implements Glassy, so that the logger will use the class name.
 */
public class RotaryPositionSubsystem1d extends SubsystemBase implements Glassy {
    /**
     * It's useful for a subsystem to know about positional settings, so that all
     * the knowledge about the subsystem itself is contained here.
     */
    private static final double THE_SPECIAL_SPOT = 1.5;

    /**
     * This should be the total reduction from the motor shaft to the mechanism
     * angle.
     */
    private static final int GEAR_RATIO = 25;
    /**
     * Most mechanisms have physical limits, and you should enter them here, to keep
     * the controller from trying to self-destruct. If your mechanism spins freely,
     * you can enter Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY here.
     */
    private static final double MIN_POSITION = -0.5;
    private static final double MAX_POSITION = 4;

    private final AngularPositionServo m_servo;

    public RotaryPositionSubsystem1d(LoggerFactory parent) {
        LoggerFactory log = parent.child(this);

        double positionGain = 4.0;
        double velocityGain = 0.11;
        double positionTolerance = 0.05;
        double velocityTolerance = 0.05;
        Feedback100 feedback = new FullStateFeedback(
                log, positionGain, velocityGain, MathUtil::angleModulus, positionTolerance, velocityTolerance);

        double maxVel = 40;
        double maxAccel = 40;
        double maxJerk = 70;
        JerkLimitedProfile100 profile = new JerkLimitedProfile100(maxVel, maxAccel, maxJerk, true);
        ProfileReference1d ref = new TimedProfileReference1d(profile);

        /*
         * Here we use the Team 100 "Identity" mechanism to allow different
         * configurations for different hardware. The most important distinction here is
         * for simulation.
         */
        switch (Identity.instance) {
            case COMP_BOT -> {
                // these constants only apply to the COMP_BOT case.
                // note the pattern here: using a variable is a way to label the thing
                // without needing to write a comment.
                int canID = 1;
                int supplyLimit = 60;
                int statorLimit = 90;
                int sensorChannel = 5;
                double inputOffset = 0.135541;
                PIDConstants PID = PIDConstants.makeVelocityPID(0.3);
                // you should make a case in the feedforward class for your constants
                Feedforward100 FF = Feedforward100.makeSimple();
                Kraken6Motor motor = new Kraken6Motor(
                        log, canID, MotorPhase.REVERSE, supplyLimit, statorLimit, PID, FF);
                RotaryPositionSensor sensor = new AS5048RotaryPositionSensor(
                        log, sensorChannel, inputOffset, EncoderDrive.DIRECT, false);
                RotaryMechanism mech = new RotaryMechanism(
                        log, motor, sensor, GEAR_RATIO, MIN_POSITION, MAX_POSITION);
                m_servo = new OnboardAngularPositionServo(
                        log, mech, ref, feedback);
                m_servo.reset();
            }
            default -> {
                SimulatedBareMotor motor = new SimulatedBareMotor(log, 600);
                SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
                SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                        log, encoder, GEAR_RATIO);
                RotaryMechanism mech = new RotaryMechanism(
                        log, motor, sensor, GEAR_RATIO, MIN_POSITION, MAX_POSITION);
                m_servo = new OnboardAngularPositionServo(
                        log, mech, ref, feedback);
                m_servo.reset();
            }
        }
    }

    ///////////////////////////////////////////////////////
    //
    // ACTIONS
    //
    // These methods make the subsystem do something.

    public void setPositionProfiled(double goal) {
        m_servo.setPositionProfiled(goal, 0);
    }

    public void setPositionDirect(Setpoints1d setpoint) {
        m_servo.setPositionDirect(setpoint, 0);
    }

    ///////////////////////////////////////////////////////
    //
    // COMMANDS
    //
    // For single-subsystem actions, these actuator commands are the cleanest way to
    // do it. Coordinated multi-subsystem actions would need to use the methods
    // above.
    //

    /** Return to the "home" position, forever. */
    public Command goHome() {
        return run(() -> {
            setPositionProfiled(0);
        });
    }

    /** Go to the spot with a profile, forever. */
    public Command goToTheSpot() {
        return run(() -> {
            setPositionProfiled(THE_SPECIAL_SPOT);
        });
    }

    /**
     * Used by "until" to end the command. For simple end-conditions, this is good
     * enough.
     */
    public boolean isDone() {
        return m_servo.atGoal();
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

}
