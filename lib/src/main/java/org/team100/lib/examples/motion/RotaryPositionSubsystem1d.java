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
 * 
 */
public class RotaryPositionSubsystem1d extends SubsystemBase implements Glassy {
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

    private final AngularPositionServo servo;

    public RotaryPositionSubsystem1d(LoggerFactory parent) {
        LoggerFactory log = parent.child(this);

        Feedback100 wristFeedback = new FullStateFeedback(
                log, 4.0, 0.11, x -> x, 0.05, 0.05);

        double maxVel = 40;
        double maxAccel = 40;
        double maxJerk = 70;
        JerkLimitedProfile100 profile = new JerkLimitedProfile100(maxVel, maxAccel, maxJerk, false);
        ProfileReference1d ref = new TimedProfileReference1d(profile);

        /*
         * Here we use the Team 100 "Identity" mechanism to allow different
         * configurations for different hardware. The most important distinction here is
         * for simulation.
         */
        switch (Identity.instance) {
            case COMP_BOT -> {
                // these constants only apply to the COMP_BOT case
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
                servo = new OnboardAngularPositionServo(
                        log, mech, ref, wristFeedback);
                servo.reset();
            }
            default -> {
                SimulatedBareMotor motor = new SimulatedBareMotor(log, 600);
                SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
                SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                        log, encoder, GEAR_RATIO);
                RotaryMechanism mech = new RotaryMechanism(
                        log, motor, sensor, GEAR_RATIO, MIN_POSITION, MAX_POSITION);
                servo = new OnboardAngularPositionServo(
                        log, mech, ref, wristFeedback);
                servo.reset();
            }
        }
    }

    public void setPositionProfiled(double goal) {
        servo.setPositionProfiled(goal, 0);
    }

    public void setPositionDirect(Setpoints1d setpoint) {
        servo.setPositionDirect(setpoint, 0);
    }

    @Override
    public void periodic() {
        servo.periodic();
    }

    /**
     * Go to the goal with a profile, forever. For simple "go to position" commands,
     * you can use this factory instead of writing a Command class.
     */
    public Command setProfiled(double goal) {
        return this.run(() -> setPositionProfiled(goal));
    }

    /**
     * Used by "until" to end the command. For simple end-conditions, this is good
     * enough.
     */
    public boolean isDone() {
        return servo.atGoal();
    }

}
