package org.team100.lib.subsystems.swerve.module;

import java.util.function.Supplier;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.ctre.Falcon6Motor;
import org.team100.lib.motor.ctre.Kraken6Motor;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.sensor.position.absolute.CombinedRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.RotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.wpi.AS5048RotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.ctre.Talon6Encoder;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.LinearVelocityServo;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;

public class WCPSwerveModule100 extends SwerveModule100 {
    private static final double STEERING_POSITION_TOLERANCE_RAD = 0.05;
    private static final double STEERING_VELOCITY_TOLERANCE_RAD_S = 0.05;
    // https://github.com/frc1678/C2024-Public/blob/17e78272e65a6ce4f87c00a3514c79f787439ca1/src/main/java/com/team1678/frc2024/Constants.java#L212
    // 2/26/25 Joel increased the steering limits *a lot*, they were 10/20, now
    // 60/80, which may mean it's more imporant now to avoid twitching and
    // oscillating.
    private static final double STEERING_SUPPLY_LIMIT = 60;
    private static final double STEERING_STATOR_LIMIT = 80;
    /**
     * WCP calls this "rotation ratio" here, we use the "flipped belt" which is the
     * fastest steering ratio.
     * 12t -> 24t
     * 14t -> 72t
     * = 72 / 7
     * https://docs.wcproducts.com/wcp-swervex/misc/other-configurations/ratio-options
     */
    private static final double STEERING_RATIO = 10.28571429;

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
    private static final double WHEEL_DIAMETER_M = 0.094; // 0.1015

    /**
     * MAKE SURE THAT THE BEVELS ON THE WHEELS FOR ZEROING GO TO THE RIGHT
     */
    public static WCPSwerveModule100 getKrakenDrive(
            LoggerFactory parent,
            double supplyLimitAmps,
            double statorLimitAmps,
            CanId driveMotorCanId,
            DriveRatio ratio,
            CanId turningMotorCanId,
            RoboRioChannel turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics,
            EncoderDrive drive,
            NeutralMode neutral,
            MotorPhase motorPhase) {

        LinearVelocityServo driveServo = driveKrakenServo(
                parent.name("Drive"),
                supplyLimitAmps,
                statorLimitAmps,
                driveMotorCanId,
                ratio);
        AngularPositionServo turningServo = turningServo(
                parent.name("Turning"),
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                STEERING_RATIO,
                kinodynamics,
                drive,
                neutral,
                motorPhase);
        return new WCPSwerveModule100(driveServo, turningServo, ratio);
    }

    /**
     * MAKE SURE THAT THE BEVELS ON THE WHEELS FOR ZEROING GO TO THE RIGHT
     */
    public static WCPSwerveModule100 getFalconDrive(
            LoggerFactory parent,
            double supplyLimitAmps,
            double statorLimitAmps,
            CanId driveMotorCanId,
            DriveRatio ratio,
            CanId turningMotorCanId,
            RoboRioChannel turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics,
            EncoderDrive drive,
            NeutralMode neutral,
            MotorPhase motorPhase) {
        LinearVelocityServo driveServo = driveFalconServo(
                parent.name("Drive"),
                supplyLimitAmps,
                statorLimitAmps,
                driveMotorCanId,
                ratio);
        AngularPositionServo turningServo = turningServo(
                parent.name("Turning"),
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                STEERING_RATIO,
                kinodynamics,
                drive,
                neutral,
                motorPhase);
        return new WCPSwerveModule100(driveServo, turningServo, ratio);
    }

    private static LinearVelocityServo driveKrakenServo(
            LoggerFactory parent,
            double supplyLimit,
            double statorLimit,
            CanId driveMotorCanId,
            DriveRatio ratio) {
        Feedforward100 ff = Feedforward100.makeWCPSwerveDriveKraken6(parent);
        // note (10/2/24) 0.4 produces oscillation, on carpet.
        PIDConstants pid = PIDConstants.makeVelocityPID(parent, 0.3);
        Kraken6Motor driveMotor = new Kraken6Motor(
                parent,
                driveMotorCanId,
                NeutralMode.COAST,
                MotorPhase.FORWARD,
                supplyLimit,
                statorLimit,
                pid,
                ff);
        Talon6Encoder encoder = driveMotor.encoder();
        LinearMechanism mech = new LinearMechanism(parent,
                driveMotor,
                encoder,
                ratio.m_ratio,
                WHEEL_DIAMETER_M,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        return new OutboardLinearVelocityServo(parent, mech);
    }

    private static LinearVelocityServo driveFalconServo(
            LoggerFactory parent,
            double supplyLimit,
            double statorLimit,
            CanId driveMotorCanId,
            DriveRatio ratio) {
        Feedforward100 ff = Feedforward100.makeWCPSwerveDriveFalcon6(parent);
        PIDConstants pid = PIDConstants.makeVelocityPID(parent, 0.3);
        Falcon6Motor driveMotor = new Falcon6Motor(
                parent,
                driveMotorCanId,
                NeutralMode.COAST,
                MotorPhase.FORWARD,
                supplyLimit,
                statorLimit,
                pid,
                ff);
        Talon6Encoder encoder = driveMotor.encoder();
        LinearMechanism mech = new LinearMechanism(parent,
                driveMotor, encoder, ratio.m_ratio, WHEEL_DIAMETER_M, Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        return new OutboardLinearVelocityServo(
                parent,
                mech);
    }

    private static AngularPositionServo turningServo(
            LoggerFactory parent,
            CanId turningMotorCanId,
            RoboRioChannel turningEncoderChannel,
            double turningOffset,
            double gearRatio,
            SwerveKinodynamics kinodynamics,
            EncoderDrive drive,
            NeutralMode neutral,
            MotorPhase motorPhase) {

        // Talon outboard POSITION PID
        // 10/2/24 drive torque produces about a 0.5 degree deviation so maybe
        // this is too low.
        PIDConstants lowLevelPID = PIDConstants.makePositionPID(parent, 10.0);

        // java uses this to calculate feedforward voltages from target velocities etc
        Feedforward100 ff = Feedforward100.makeWCPSwerveTurningFalcon6(parent);

        Falcon6Motor turningMotor = new Falcon6Motor(
                parent,
                turningMotorCanId,
                neutral,
                motorPhase,
                STEERING_SUPPLY_LIMIT,
                STEERING_STATOR_LIMIT,
                lowLevelPID,
                ff);

        // this reads the steering angle directly.
        RotaryPositionSensor turningSensor = new AS5048RotaryPositionSensor(
                parent,
                turningEncoderChannel,
                turningOffset,
                drive);

        Talon6Encoder builtInEncoder = turningMotor.encoder();

        ProxyRotaryPositionSensor proxy = new ProxyRotaryPositionSensor(builtInEncoder, gearRatio);
        CombinedRotaryPositionSensor combined = new CombinedRotaryPositionSensor(parent, turningSensor, proxy);

        RotaryMechanism mech = new RotaryMechanism(
                parent, turningMotor, combined, gearRatio, Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);

        AngularPositionServo turningServo = outboardTurningServo(
                parent, kinodynamics, mech, combined);
        turningServo.reset();
        return turningServo;
    }

    private static AngularPositionServo outboardTurningServo(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            RotaryMechanism mech,
            CombinedRotaryPositionSensor combinedEncoder) {
        Supplier<IncrementalProfile> profile = kinodynamics.getSteeringProfile();
        ProfileReferenceR1 ref = new IncrementalProfileReferenceR1(
                parent, profile, STEERING_POSITION_TOLERANCE_RAD, STEERING_VELOCITY_TOLERANCE_RAD_S);
        return new OutboardAngularPositionServo(parent, mech, ref);
    }

    private WCPSwerveModule100(
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo,
            DriveRatio ratio) {
        // primary is 2:1 so final is whatever is left.
        super(driveServo, turningServo, WHEEL_DIAMETER_M, ratio.m_ratio / 2);
        //
    }
}
