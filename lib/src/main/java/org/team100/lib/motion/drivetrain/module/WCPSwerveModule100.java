package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.AnalogTurningEncoder;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.DutyCycleRotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.profile.Profile100;

public class WCPSwerveModule100 extends SwerveModule100 {
    private static final double kSteeringSupplyLimit = 10;
    private static final double kSteeringStatorLimit = 20;
    /**
     * WCP calls this "rotation ratio" here, we use the "flipped belt" which is the
     * fastest steering ratio.
     * 12t -> 24t
     * 14t -> 72t
     * = 72 / 7
     * https://docs.wcproducts.com/wcp-swervex/misc/other-configurations/ratio-options
     */
    private static final double kSteeringRatio = 10.28571429;

    /**
     * Flipped belt ratios.
     * 
     * See
     * https://docs.wcproducts.com/wcp-swervex/misc/other-configurations/ratio-options
     */
    public enum DriveRatio {
        FAST(5.5),
        MEDIUM(6.55);

        private double m_ratio;

        DriveRatio(double ratio) {
            m_ratio = ratio;
        }
    }

    // WCP 4 inch wheel
    private static final double kWheelDiameterM = 0.095; // 0.1015

    /**
     * MAKE SURE THAT THE BEVELS ON THE WHEELS FOR ZEROING GO TO THE RIGHT
     */
    public static WCPSwerveModule100 getKrakenDrive(
            LoggerFactory parent,
            double supplyLimitAmps,
            double statorLimitAmps,
            int driveMotorCanId,
            DriveRatio ratio,
            Class<? extends RotaryPositionSensor> encoderClass,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics,
            EncoderDrive drive,
            MotorPhase motorPhase) {

        LinearVelocityServo driveServo = driveKrakenServo(
                parent.child("Drive"),
                supplyLimitAmps,
                statorLimitAmps,
                driveMotorCanId,
                ratio);
        AngularPositionServo turningServo = turningServo(
                parent.child("Turning"),
                encoderClass,
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                kSteeringRatio,
                kinodynamics,
                drive,
                motorPhase);

        return new WCPSwerveModule100(driveServo, turningServo);
    }

    /**
     * MAKE SURE THAT THE BEVELS ON THE WHEELS FOR ZEROING GO TO THE RIGHT
     */
    public static WCPSwerveModule100 getFalconDrive(
            LoggerFactory parent,
            double supplyLimitAmps,
            double statorLimitAmps,
            int driveMotorCanId,
            DriveRatio ratio,
            Class<? extends RotaryPositionSensor> encoderClass,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics,
            EncoderDrive drive,
            MotorPhase motorPhase) {
        LinearVelocityServo driveServo = driveFalconServo(
                parent.child("Drive"),
                supplyLimitAmps,
                statorLimitAmps,
                driveMotorCanId,
                ratio);
        AngularPositionServo turningServo = turningServo(
                parent.child("Turning"),
                encoderClass,
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                kSteeringRatio,
                kinodynamics,
                drive,
                motorPhase);
        return new WCPSwerveModule100(driveServo, turningServo);
    }

    private static LinearVelocityServo driveKrakenServo(
            LoggerFactory parent,
            double supplyLimit,
            double statorLimit,
            int driveMotorCanId,
            DriveRatio ratio) {
        Feedforward100 ff = Feedforward100.makeWCPSwerveDriveFalcon6();
        // note (10/2/24) 0.4 produces oscillation, on carpet.
        PIDConstants pid = new PIDConstants(0.3);
        Kraken6Motor driveMotor = new Kraken6Motor(
                parent,
                driveMotorCanId,
                MotorPhase.FORWARD,
                supplyLimit,
                statorLimit,
                pid,
                ff);
        LinearMechanism mech = new SimpleLinearMechanism(
                driveMotor,
                new Talon6Encoder(parent, driveMotor),
                ratio.m_ratio,
                kWheelDiameterM);
        return new OutboardLinearVelocityServo(
                parent,
                mech);
    }

    private static LinearVelocityServo driveFalconServo(
            LoggerFactory parent,
            double supplyLimit,
            double statorLimit,
            int driveMotorCanId,
            DriveRatio ratio) {
        Feedforward100 ff = Feedforward100.makeWCPSwerveDriveFalcon6();
        PIDConstants pid = new PIDConstants(0.2);
        Falcon6Motor driveMotor = new Falcon6Motor(
                parent,
                driveMotorCanId,
                MotorPhase.FORWARD,
                supplyLimit,
                statorLimit,
                pid,
                ff);
        LinearMechanism mech = new SimpleLinearMechanism(
                driveMotor,
                new Talon6Encoder(parent, driveMotor),
                ratio.m_ratio,
                kWheelDiameterM);
        return new OutboardLinearVelocityServo(
                parent,
                mech);
    }

    private static AngularPositionServo turningServo(
            LoggerFactory parent,
            Class<? extends RotaryPositionSensor> encoderClass,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            double gearRatio,
            SwerveKinodynamics kinodynamics,
            EncoderDrive drive,
            MotorPhase motorPhase) {

        // Talon outboard POSITION PID
        // 10/2/24 drive torque produces about a 0.5 degree deviation so maybe
        // this is too low.
        PIDConstants lowLevelPID = new PIDConstants(10.0, 0.0, 0.0);

        // java uses this to calculate feedforward voltages from target velocities etc
        Feedforward100 ff = Feedforward100.makeWCPSwerveTurningFalcon6();

        Falcon6Motor turningMotor = new Falcon6Motor(
                parent,
                turningMotorCanId,
                motorPhase,
                kSteeringSupplyLimit,
                kSteeringStatorLimit,
                lowLevelPID,
                ff);

        RotaryPositionSensor turningEncoder = turningEncoder(
                encoderClass,
                parent,
                turningEncoderChannel,
                turningOffset,
                drive);

        Profile100 profile = kinodynamics.getSteeringProfile();

        Talon6Encoder builtInEncoder = new Talon6Encoder(
                parent,
                turningMotor);

        RotaryMechanism mech = new SimpleRotaryMechanism(
                parent,
                turningMotor,
                builtInEncoder,
                gearRatio);

        AngularPositionServo turningServo = getOutboard(
                parent,
                turningEncoder,
                profile,
                mech);
        turningServo.reset();
        return turningServo;
    }

    private static AngularPositionServo getOutboard(
            LoggerFactory parent,
            RotaryPositionSensor turningEncoder,
            Profile100 profile,
            RotaryMechanism mech) {
        CombinedEncoder combinedEncoder = new CombinedEncoder(
                parent,
                turningEncoder,
                mech);
        AngularPositionServo servo = new OutboardAngularPositionServo(
                parent,
                mech,
                combinedEncoder,
                profile);
        return servo;
    }

    private static RotaryPositionSensor turningEncoder(
            Class<? extends RotaryPositionSensor> encoderClass,
            LoggerFactory parent,
            int channel,
            double inputOffset,
            EncoderDrive drive) {
        if (encoderClass == AnalogTurningEncoder.class) {
            return new AnalogTurningEncoder(
                    parent,
                    channel,
                    inputOffset,
                    drive);
        }
        if (encoderClass == DutyCycleRotaryPositionSensor.class) {
            return new AS5048RotaryPositionSensor(
                    parent,
                    channel,
                    inputOffset,
                    drive);
        }
        throw new IllegalArgumentException("unknown encoder class: " + encoderClass.getName());
    }

    private WCPSwerveModule100(
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        super(driveServo, turningServo);
        //
    }
}
