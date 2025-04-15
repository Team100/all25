package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.encoder.CombinedRotaryPositionSensor;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.reference.IncrementalProfileReference1d;

/**
 * Uses simulated position sensors, must be used with clock control (e.g.
 * {@link Timeless}).
 */
public class SimulatedSwerveModule100 extends SwerveModule100 {

    public static SimulatedSwerveModule100 get(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics) {
        LinearVelocityServo driveServo = simulatedDriveServo(
                parent.child("Drive"));
        AngularPositionServo turningServo = simulatedTurningServo(
                parent.child("Turning"),
                kinodynamics);
        return new SimulatedSwerveModule100(driveServo, turningServo);
    }

    /**
     * The simulated outboard servo instantaneously obeys position input
     */
    public static SimulatedSwerveModule100 withInstantaneousSteering(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics) {
        LinearVelocityServo driveServo = simulatedDriveServo(
                parent.child("Drive"));
        AngularPositionServo turningServo = simulatedOutboardTurningServo(
                parent.child("Turning"),
                kinodynamics);
        return new SimulatedSwerveModule100(driveServo, turningServo);
    }

    private static LinearVelocityServo simulatedDriveServo(LoggerFactory parent) {
        // simulated drive motor free speed is 5 m/s
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        // simulated gearing is 2 meter wheel, 1:1, so rad/s and m/s are the same.
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(parent, driveMotor);
        LinearMechanism mech = new LinearMechanism(
                driveMotor, encoder, 1, 2, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        return new OutboardLinearVelocityServo(parent, mech);
    }

    /**
     * Uses simulated position sensors, must be used with clock control (e.g.
     * {@link Timeless}).
     */
    private static AngularPositionServo simulatedTurningServo(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics) {
        // simulated turning motor free speed is 20 rad/s
        SimulatedBareMotor turningMotor = new SimulatedBareMotor(parent, 20);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(parent, turningMotor);
        SimulatedRotaryPositionSensor turningSensor = new SimulatedRotaryPositionSensor(
                parent, encoder, 1, () -> 0);
        RotaryMechanism turningMech = new RotaryMechanism(
                parent, turningMotor, turningSensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        Feedback100 turningPositionFeedback = new PIDFeedback(
                parent,
                20, // kP
                0, // kI
                0, // kD
                true,
                0.05, // note low tolerance
                1);
        Profile100 profile = kinodynamics.getSteeringProfile();
        IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);
        OnboardAngularPositionServo turningServo = new OnboardAngularPositionServo(
                parent,
                turningMech,
                ref,
                turningPositionFeedback);
        turningServo.reset();
        return turningServo;
    }

    /**
     * Simulates direct (instant) positional control, which avoids the problem
     * where the steering profiles don't produce feedforwards for high speeds and
     * small errors.
     */
    private static AngularPositionServo simulatedOutboardTurningServo(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics) {
        // simulated turning motor free speed is 20 rad/s
        SimulatedBareMotor motor = new SimulatedBareMotor(parent, 20);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(parent, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                parent, encoder, 1, () -> 0);

        ProxyRotaryPositionSensor proxy = new ProxyRotaryPositionSensor(encoder, 1);
        CombinedRotaryPositionSensor combinedEncoder = new CombinedRotaryPositionSensor(
                parent, sensor, proxy);

        RotaryMechanism turningMech = new RotaryMechanism(
                parent, motor, combinedEncoder, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        Profile100 profile = kinodynamics.getSteeringProfile();
        IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);

        OutboardAngularPositionServo turningServo = new OutboardAngularPositionServo(
                parent, turningMech, ref);
        turningServo.reset();
        return turningServo;
    }

    private SimulatedSwerveModule100(
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        super(driveServo, turningServo);
        //
    }

}
