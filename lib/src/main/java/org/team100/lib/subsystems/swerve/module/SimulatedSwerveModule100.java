package org.team100.lib.subsystems.swerve.module;

import java.util.function.Supplier;

import org.team100.lib.controller.r1.Feedback100;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.sensor.position.absolute.CombinedRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.LinearVelocityServo;
import org.team100.lib.servo.OnboardAngularPositionServo;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;

/**
 * Uses simulated position sensors, must be used with clock control (e.g.
 * {@link Timeless}).
 */
public class SimulatedSwerveModule100 extends SwerveModule100 {

    private static final double DRIVE_GEAR_RATIO = 5.5;
    private static final double WHEEL_DIAMETER_M = 0.1;

    public static SimulatedSwerveModule100 get(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics) {
        LinearVelocityServo driveServo = simulatedDriveServo(
                parent.name("Drive"));
        AngularPositionServo turningServo = simulatedTurningServo(
                parent.name("Turning"),
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
                parent.name("Drive"));
        AngularPositionServo turningServo = simulatedOutboardTurningServo(
                parent.name("Turning"),
                kinodynamics);
        return new SimulatedSwerveModule100(driveServo, turningServo);
    }

    private static LinearVelocityServo simulatedDriveServo(LoggerFactory parent) {
        SimulatedBareMotor motor = new SimulatedBareMotor(parent, 600);
        return OutboardLinearVelocityServo.make(
                parent, motor, DRIVE_GEAR_RATIO, WHEEL_DIAMETER_M);
    }

    /**
     * Uses simulated position sensors, must be used with clock control (e.g.
     * {@link Timeless}).
     */
    private static AngularPositionServo simulatedTurningServo(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics) {
        // simulated turning motor free speed is 20 rad/s
        SimulatedBareMotor turningMotor = new SimulatedBareMotor(parent, 600);
        IncrementalBareEncoder encoder = turningMotor.encoder();
        SimulatedRotaryPositionSensor turningSensor = new SimulatedRotaryPositionSensor(
                parent, encoder, 1);
        RotaryMechanism turningMech = new RotaryMechanism(
                parent, turningMotor, turningSensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        Feedback100 turningPositionFeedback = new PIDFeedback(
                parent,
                10, // kP .. was 20
                0, // kI
                0, // kD
                false,
                0.05, // note low tolerance
                1);
        Supplier<IncrementalProfile> profile = kinodynamics.getSteeringProfile();
        // without a profile, there's no velocity feedforward. Hm.
        IncrementalProfileReferenceR1 ref = new IncrementalProfileReferenceR1(parent, profile, 0.05, 0.05);
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
        SimulatedBareMotor motor = new SimulatedBareMotor(parent, 600);
        IncrementalBareEncoder encoder = motor.encoder();
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                parent, encoder, 1);

        ProxyRotaryPositionSensor proxy = new ProxyRotaryPositionSensor(encoder, 1);
        CombinedRotaryPositionSensor combinedEncoder = new CombinedRotaryPositionSensor(
                parent, sensor, proxy);

        RotaryMechanism turningMech = new RotaryMechanism(
                parent, motor, combinedEncoder, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        Supplier<IncrementalProfile> profile = kinodynamics.getSteeringProfile();
        IncrementalProfileReferenceR1 ref = new IncrementalProfileReferenceR1(parent, profile, 0.05, 0.05);

        OutboardAngularPositionServo turningServo = new OutboardAngularPositionServo(
                parent, turningMech, ref);
        turningServo.reset();
        return turningServo;
    }

    private SimulatedSwerveModule100(
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        // primary is 2:1 so final is whatever is left.
        super(driveServo, turningServo, WHEEL_DIAMETER_M, DRIVE_GEAR_RATIO / 2);
        //
    }
}
