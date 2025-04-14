package org.team100.lib.motion.drivetrain.module;

import java.util.OptionalDouble;

import org.team100.lib.config.Identity;
import org.team100.lib.encoder.DutyCycleRotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePositions;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.motion.drivetrain.module.WCPSwerveModule100.DriveRatio;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.util.Util;

/**
 * Represents the modules in the drivetrain.
 * Do not put logic here; this is just for bundling the modules together.
 */
public class SwerveModuleCollection {
    private static final boolean DEBUG = false;
    private static final String kSwerveModules = "Swerve Modules";
    private static final String kFrontLeft = "Front Left";
    private static final String kFrontRight = "Front Right";
    private static final String kRearLeft = "Rear Left";
    private static final String kRearRight = "Rear Right";

    private final SwerveModule100 m_frontLeft;
    private final SwerveModule100 m_frontRight;
    private final SwerveModule100 m_rearLeft;
    private final SwerveModule100 m_rearRight;

    public SwerveModuleCollection(
            SwerveModule100 frontLeft,
            SwerveModule100 frontRight,
            SwerveModule100 rearLeft,
            SwerveModule100 rearRight) {
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_rearLeft = rearLeft;
        m_rearRight = rearRight;
    }

    /**
     * Creates collections according to Identity.
     */
    public static SwerveModuleCollection get(
            LoggerFactory parent,
            double currentLimit,
            double statorLimit,
            SwerveKinodynamics kinodynamics) {
        LoggerFactory collectionLogger = parent.child(kSwerveModules);
        LoggerFactory frontLeftLogger = collectionLogger.child(kFrontLeft);
        LoggerFactory frontRightLogger = collectionLogger.child(kFrontRight);
        LoggerFactory rearLeftLogger = collectionLogger.child(kRearLeft);
        LoggerFactory rearRightLogger = collectionLogger.child(kRearRight);

        switch (Identity.instance) {
            case COMP_BOT:
                Util.println("************** WCP MODULES w/Duty-Cycle Encoders **************");
                return new SwerveModuleCollection(
                        WCPSwerveModule100.getKrakenDrive(frontLeftLogger,
                                currentLimit,
                                statorLimit,
                                18,
                                DriveRatio.MEDIUM, DutyCycleRotaryPositionSensor.class,
                                55,
                                0,
                                0.512104,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(frontRightLogger,
                                currentLimit,
                                statorLimit,
                                56,
                                DriveRatio.MEDIUM, DutyCycleRotaryPositionSensor.class,
                                51,
                                1,
                                0.512468,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(rearLeftLogger,
                                currentLimit,
                                statorLimit,
                                22,
                                DriveRatio.MEDIUM, DutyCycleRotaryPositionSensor.class,
                                52,
                                2,
                                0.184939,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(rearRightLogger,
                                currentLimit,
                                statorLimit,
                                4,
                                DriveRatio.MEDIUM, DutyCycleRotaryPositionSensor.class,
                                21,
                                7,
                                0.954909,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE));
            case SWERVE_ONE:
                Util.println("************** WCP MODULES w/Duty-Cycle Encoders **************");
                return new SwerveModuleCollection(
                        WCPSwerveModule100.getFalconDrive(frontLeftLogger,
                                currentLimit,
                                statorLimit,
                                32,
                                DriveRatio.FAST, DutyCycleRotaryPositionSensor.class,
                                12,
                                7,
                                0.658,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.getFalconDrive(frontRightLogger,
                                currentLimit,
                                statorLimit,
                                30,
                                DriveRatio.FAST, DutyCycleRotaryPositionSensor.class,
                                11,
                                8,
                                0.379,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.getFalconDrive(rearLeftLogger,
                                currentLimit,
                                statorLimit,
                                31,
                                DriveRatio.FAST, DutyCycleRotaryPositionSensor.class,
                                21,
                                6,
                                0.41,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.getFalconDrive(rearRightLogger,
                                currentLimit,
                                statorLimit,
                                22,
                                DriveRatio.FAST, DutyCycleRotaryPositionSensor.class,
                                33,
                                9,
                                0.03,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE));
            case BETA_BOT:
            case SWERVE_TWO:
            case BLANK:
            default:
                if (DEBUG)
                    Util.println("************** SIMULATED MODULES **************");
                /*
                 * Uses simulated position sensors, must be used with clock control (e.g.
                 * {@link Timeless}).
                 */
                return new SwerveModuleCollection(
                        SimulatedSwerveModule100.get(frontLeftLogger, kinodynamics),
                        SimulatedSwerveModule100.get(frontRightLogger, kinodynamics),
                        SimulatedSwerveModule100.get(rearLeftLogger, kinodynamics),
                        SimulatedSwerveModule100.get(rearRightLogger, kinodynamics));
        }
    }

    //////////////////////////////////////////////////
    //
    // Actuators
    //

    /**
     * Optimizes.
     * 
     * Works fine with empty angles.
     * 
     * @param swerveModuleStates
     */
    public void setDesiredStates(SwerveModuleStates swerveModuleStates) {
        if (DEBUG)
            Util.printf("setDesiredStates() %s\n", swerveModuleStates);
        m_frontLeft.setDesiredState(swerveModuleStates.frontLeft());
        m_frontRight.setDesiredState(swerveModuleStates.frontRight());
        m_rearLeft.setDesiredState(swerveModuleStates.rearLeft());
        m_rearRight.setDesiredState(swerveModuleStates.rearRight());
    }

    /**
     * Does not optimize.
     * 
     * This "raw" mode is just for testing.
     * 
     * Works fine with empty angles.
     */
    public void setRawDesiredStates(SwerveModuleStates swerveModuleStates) {
        m_frontLeft.setRawDesiredState(swerveModuleStates.frontLeft());
        m_frontRight.setRawDesiredState(swerveModuleStates.frontRight());
        m_rearLeft.setRawDesiredState(swerveModuleStates.rearLeft());
        m_rearRight.setRawDesiredState(swerveModuleStates.rearRight());
    }

    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_rearLeft.stop();
        m_rearRight.stop();
    }

    public void reset() {
        m_frontLeft.reset();
        m_frontRight.reset();
        m_rearLeft.reset();
        m_rearRight.reset();
    }

    //////////////////////////////////////////////////////
    //
    // Observers
    //

    public SwerveModulePositions positions() {
        return new SwerveModulePositions(
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition());
    }

    public OptionalDouble[] turningPosition() {
        return new OptionalDouble[] {
                m_frontLeft.turningPosition(),
                m_frontRight.turningPosition(),
                m_rearLeft.turningPosition(),
                m_rearRight.turningPosition()
        };
    }

    /** FOR TEST ONLY */
    public SwerveModuleStates states() {
        return new SwerveModuleStates(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
    }

    public boolean[] atSetpoint() {
        return new boolean[] {
                m_frontLeft.atSetpoint(),
                m_frontRight.atSetpoint(),
                m_rearLeft.atSetpoint(),
                m_rearRight.atSetpoint()
        };
    }

    ////////////////////////////////////////////

    public void close() {
        m_frontLeft.close();
        m_frontRight.close();
        m_rearLeft.close();
        m_rearRight.close();
    }

    public SwerveModule100[] modules() {
        return new SwerveModule100[] {
                m_frontLeft,
                m_frontRight,
                m_rearLeft,
                m_rearRight };
    }

    /** Updates visualization. */
    public void periodic() {

        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_rearLeft.periodic();
        m_rearRight.periodic();
    }
}
