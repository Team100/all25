package org.team100.lib.motion.arm23;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.AnalogTurningEncoder;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.CANSparkMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;

/**
 * Produces real or simulated arm subsystems depending on identity.
 */
public class ArmFactory23 {
    private static final String kArm = "arm";
    private static final String kLower = "arm/lower";
    private static final String kUpper = "arm/upper";

    public static ArmSubsystem23 get(LoggerFactory parent) {
        switch (Identity.instance) {
            case TEST_BOARD_6B:
                return real(parent);
            case BLANK:
            default:
                return simulated(parent);
        }
    }

    private static ArmSubsystem23 real(LoggerFactory parent) {
        final double kLowerEncoderOffset = 0.861614;
        final double kUpperEncoderOffset = 0.266396;
        final double kReduction = 600;

        LoggerFactory lowerLogger = parent.child(kLower);
        CANSparkMotor lowerMotor = new NeoCANSparkMotor(
                lowerLogger,
                4,
                MotorPhase.FORWARD,
                8, // you'll want to change the current limit
                Feedforward100.makeNeo(),
                new PIDConstants());

        RotaryPositionSensor lowerSensor = new AnalogTurningEncoder(
                lowerLogger,
                1, // analog input 1
                kLowerEncoderOffset,
                EncoderDrive.INVERSE);

        RotaryMechanism lowerMech = new RotaryMechanism(
                lowerLogger, lowerMotor, lowerSensor,
                kReduction, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        LoggerFactory upperLogger = parent.child(kUpper);
        CANSparkMotor upperMotor = new NeoCANSparkMotor(
                upperLogger,
                30,
                MotorPhase.FORWARD,
                1, // you'll want to change the current limit
                Feedforward100.makeNeo(),
                new PIDConstants());

        RotaryPositionSensor upperSensor = new AnalogTurningEncoder(
                upperLogger,
                0, // analog input 0
                kUpperEncoderOffset,
                EncoderDrive.DIRECT);

        RotaryMechanism upperMech = new RotaryMechanism(
                upperLogger, upperMotor, upperSensor,
                kReduction, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        return new ArmSubsystem23(
                parent.child(kArm),
                lowerMech,
                lowerSensor,
                upperMech,
                upperSensor);
    }

    /**
     * Uses simulated position sensors, must be used with clock control (e.g.
     * {@link Timeless}).
     */
    private static ArmSubsystem23 simulated(LoggerFactory parent) {
        // for testing
        // note very high reduction ratio
        final double kFreeSpeedRad_S = 200;
        final double kReduction = 600;
        // motor speed is rad/s

        LoggerFactory lowerLogger = parent.child(kLower);
        SimulatedBareMotor lowerMotor = new SimulatedBareMotor(lowerLogger, kFreeSpeedRad_S);
        SimulatedBareEncoder lowerEncoder = new SimulatedBareEncoder(lowerLogger, lowerMotor);
        // limits used to be -1, 1, when we used winding encoders.
        // i don't think we ever actually *use* the limits for anything, though.
        SimulatedRotaryPositionSensor lowerSensor = new SimulatedRotaryPositionSensor(
                lowerLogger, lowerEncoder, kReduction);
        RotaryMechanism lowerMech = new RotaryMechanism(
                lowerLogger,
                lowerMotor,
                lowerSensor,
                kReduction,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);

        LoggerFactory upperLogger = parent.child(kUpper);
        SimulatedBareMotor upperMotor = new SimulatedBareMotor(upperLogger, kFreeSpeedRad_S);
        SimulatedBareEncoder upperEncoder = new SimulatedBareEncoder(upperLogger, upperMotor);
        // limits used to be 0.1, 2.5, when we used winding encoders.
        // i don't think we ever actually *use* the limits for anything, though.
        SimulatedRotaryPositionSensor upperSensor = new SimulatedRotaryPositionSensor(
                upperLogger, upperEncoder, kReduction);
        RotaryMechanism upperMech = new RotaryMechanism(
                upperLogger,
                upperMotor,
                upperSensor,
                kReduction,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        return new ArmSubsystem23(
                parent.child(kArm),
                lowerMech,
                lowerSensor,
                upperMech,
                upperSensor);
    }

    private ArmFactory23() {
        //
    }

}
