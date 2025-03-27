package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.controller.simple.IncrementalProfiledController;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motion.servo.UnprofiledOutboardAngularPositionServo;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.Profile100;

import edu.wpi.first.math.MathUtil;

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
        LinearMechanism mech = new SimpleLinearMechanism(
                driveMotor,
                new SimulatedBareEncoder(parent, driveMotor),
                1,
                2);
        return new OutboardLinearVelocityServo(
                parent,
                mech);
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
        RotaryMechanism turningMech = new SimpleRotaryMechanism(
                parent,
                turningMotor,
                new SimulatedBareEncoder(parent, turningMotor),
                1);
        SimulatedRotaryPositionSensor turningEncoder = new SimulatedRotaryPositionSensor(
                parent,
                turningMech);
        Feedback100 turningPositionFeedback = new PIDFeedback(
                parent,
                20, // kP
                0, // kI
                0, // kD
                true,
                0.05, // note low tolerance
                1);
        Profile100 profile = kinodynamics.getSteeringProfile();

        ProfiledController controller = new IncrementalProfiledController(
                parent,
                profile, turningPositionFeedback, MathUtil::angleModulus,
                0.05, 0.05);
        OnboardAngularPositionServo turningServo = new OnboardAngularPositionServo(
                parent,
                turningMech,
                turningEncoder,
                controller);
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
        SimulatedBareMotor turningMotor = new SimulatedBareMotor(parent, 20);
        RotaryMechanism turningMech = new SimpleRotaryMechanism(
                parent,
                turningMotor,
                new SimulatedBareEncoder(parent, turningMotor),
                1);
        SimulatedRotaryPositionSensor turningEncoder = new SimulatedRotaryPositionSensor(
                parent,
                turningMech);
        CombinedEncoder combinedEncoder = new CombinedEncoder(
                parent,
                turningEncoder,
                turningMech,
                false);
        UnprofiledOutboardAngularPositionServo turningServo = new UnprofiledOutboardAngularPositionServo(
                parent,
                turningMech,
                combinedEncoder);
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
