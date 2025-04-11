package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.IncrementalProfiledController;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.controller.simple.ZeroFeedback;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.AnalogTurningEncoder;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.DutyCycleRotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motion.servo.UnprofiledOutboardAngularPositionServo;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.profile.incremental.Profile100;

import edu.wpi.first.math.MathUtil;

public class WCPSwerveModule100 extends SwerveModule100 {
    // https://github.com/frc1678/C2024-Public/blob/17e78272e65a6ce4f87c00a3514c79f787439ca1/src/main/java/com/team1678/frc2024/Constants.java#L212
    // 2/26/25 Joel increased the steering limits *a lot*, they were 10/20, now
    // 60/80, which may mean it's more imporant now to avoid twitching and
    // oscillating.
    private static final double kSteeringSupplyLimit = 60;
    private static final double kSteeringStatorLimit = 80;
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
    private static final double kWheelDiameterM = 0.094; // 0.1015

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
        Feedforward100 ff = Feedforward100.makeWCPSwerveDriveKraken6();
        // note (10/2/24) 0.4 produces oscillation, on carpet.
        PIDConstants pid = PIDConstants.makeVelocityPID(0.3);
        Kraken6Motor driveMotor = new Kraken6Motor(
                parent,
                driveMotorCanId,
                MotorPhase.FORWARD,
                supplyLimit,
                statorLimit,
                pid,
                ff);
        Talon6Encoder encoder = new Talon6Encoder(parent, driveMotor);
        LinearMechanism mech = new LinearMechanism(
                driveMotor,
                encoder,
                ratio.m_ratio,
                kWheelDiameterM,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        return new OutboardLinearVelocityServo(parent, mech);
    }

    private static LinearVelocityServo driveFalconServo(
            LoggerFactory parent,
            double supplyLimit,
            double statorLimit,
            int driveMotorCanId,
            DriveRatio ratio) {
        Feedforward100 ff = Feedforward100.makeWCPSwerveDriveFalcon6();
        PIDConstants pid = PIDConstants.makeVelocityPID(0.3);
        Falcon6Motor driveMotor = new Falcon6Motor(
                parent,
                driveMotorCanId,
                MotorPhase.FORWARD,
                supplyLimit,
                statorLimit,
                pid,
                ff);
        Talon6Encoder encoder = new Talon6Encoder(parent, driveMotor);
        LinearMechanism mech = new LinearMechanism(
                driveMotor, encoder, ratio.m_ratio, kWheelDiameterM, Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
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
        PIDConstants lowLevelPID = PIDConstants.makePositionPID(10.0);

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

        // this reads the steering angle directly.
        RotaryPositionSensor turningEncoder = turningEncoder(
                encoderClass,
                parent,
                turningEncoderChannel,
                turningOffset,
                drive);

        Talon6Encoder builtInEncoder = new Talon6Encoder(parent, turningMotor);

        RotaryMechanism mech = new RotaryMechanism(
                parent, turningMotor, builtInEncoder, gearRatio, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        CombinedEncoder combinedEncoder = new CombinedEncoder(parent, turningEncoder, mech);

        AngularPositionServo turningServo = outboardTurningServo(
                parent, kinodynamics, mech, combinedEncoder);
        turningServo.reset();
        return turningServo;
    }

    private static AngularPositionServo outboardTurningServo(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            RotaryMechanism mech,
            CombinedEncoder combinedEncoder) {
        if (Experiments.instance.enabled(Experiment.UnprofiledSteering)) {
            return new UnprofiledOutboardAngularPositionServo(parent, mech, combinedEncoder);
        }
        Profile100 profile = kinodynamics.getSteeringProfile();
        Feedback100 feedback = new ZeroFeedback(x -> x, 0.02, 0.02);
        ProfiledController controller = new IncrementalProfiledController(
                parent, profile, feedback, MathUtil::angleModulus, 0.05, 0.05);
        return new OutboardAngularPositionServo(
                parent,
                mech,
                combinedEncoder,
                controller);
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
                    drive,
                    true);
        }
        if (encoderClass == DutyCycleRotaryPositionSensor.class) {
            return new AS5048RotaryPositionSensor(
                    parent,
                    channel,
                    inputOffset,
                    drive,
                    true);
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
