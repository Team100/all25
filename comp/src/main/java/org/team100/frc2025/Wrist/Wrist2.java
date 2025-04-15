package org.team100.frc2025.Wrist;

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
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.Gravity;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.Spring;
import org.team100.lib.motion.servo.Torque;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.timed.JerkLimitedProfile100;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.reference.TimedProfileReference1d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The wrist class now handles gravity torque, it's not in a "servo".
 * 
 * TODO: caller should specify the acceleration of the carriage, since that
 * affects "gravity".
 */
public class Wrist2 extends SubsystemBase implements Glassy {
    /**
     * Publish the elevator mechanism visualization to glass. This might be a little
     * bit slow, turn it off for comp.
     */
    private static final boolean VISUALIZE = true;
    private static final int GEAR_RATIO = 25;

    private static final double kWristMinimumPosition = -0.5;
    private static final double kWristMaximumPosition = 4;

    double maxVel = 40;
    double maxAccel = 40;
    double maxJerk = 70;

    // private LaserCan lc;

    private boolean m_isSafe = false;

    private final double kPositionTolerance = 0.02;
    private final double kVelocityTolerance = 0.01;
    private final AngularPositionServo wristServo;

    private final RotaryMechanism m_wristMech;
    private final Torque m_gravityAndSpringTorque;
    private final BooleanLogger m_log_safe;
    private final Runnable m_viz;

    public Wrist2(LoggerFactory parent, int wristID) {
        LoggerFactory logger = parent.child(this);

        int wristSupplyLimit = 60;
        int wristStatorLimit = 90;

        int algaeCurrentLimit = 20;
        int coralCurrentLimit = 20;

        PIDConstants wristPID = PIDConstants.makeVelocityPID(0.3); // 0.3

        Feedforward100 wristFF = Feedforward100.makeKraken6Wrist();

        Feedback100 wristFeedback = new FullStateFeedback(
                logger, 4.0, 0.11, x -> x, 0.05, 0.05);

        JerkLimitedProfile100 profile = new JerkLimitedProfile100(maxVel, maxAccel, maxJerk, false);

        ProfileReference1d ref = new TimedProfileReference1d(profile);
        // TrapezoidProfile100 profile = new TrapezoidProfile100(35, 15,
        // kPositionTolerance);

        m_log_safe = logger.booleanLogger(Level.TRACE, "Wrist Safe Condition");

        switch (Identity.instance) {
            case COMP_BOT -> {

                Kraken6Motor wristMotor = new Kraken6Motor(logger, wristID, MotorPhase.REVERSE,
                        wristSupplyLimit, wristStatorLimit, wristPID, wristFF);

                // this reads the wrist angle directly.
                RotaryPositionSensor sensor = new AS5048RotaryPositionSensor(
                        logger,
                        5,
                        0.135541, // 0.346857, //0.317012, //0.227471, //0.188726
                        EncoderDrive.DIRECT,
                        false);


                // IncrementalBareEncoder internalWristEncoder = new Talon6Encoder(logger,
                // wristMotor);

                m_wristMech = new RotaryMechanism(
                        logger,
                        wristMotor,
                        sensor,
                        GEAR_RATIO,
                        kWristMinimumPosition,
                        kWristMaximumPosition);

                // Feedback100 wristFeedback = new PIDFeedback(child, 7.5, 0.00, 0.000, false,
                // kPositionTolerance,
                // kPositionTolerance);
                // Feedback100 wristFeedback = new PIDFeedback(parent, 5.0, 0.00, 0.000, false,
                // kPositionTolerance, 0.1);
                // Feedback100 wristFeedback = new PIDFeedback(parent, 0, 0, 0 , false,
                // kPositionTolerance, kPositionTolerance);

                wristServo = new OnboardAngularPositionServo(
                        logger, m_wristMech, ref, wristFeedback);

                wristServo.reset();

                Gravity gravity = new Gravity(logger, 9.0, -0.451230);
                Spring spring = new Spring(logger);
                m_gravityAndSpringTorque = new Torque((x) -> gravity.applyAsDouble(x) + spring.applyAsDouble(x));

            }
            default -> {

                SimulatedBareMotor motor = new SimulatedBareMotor(logger, 100);
                SimulatedBareEncoder encoder = new SimulatedBareEncoder(logger, motor);
                SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                        logger, encoder, GEAR_RATIO, () -> 0);
                RotaryMechanism wristMech = new RotaryMechanism(
                        logger, motor, sensor,
                        GEAR_RATIO, kWristMinimumPosition, kWristMaximumPosition);


                wristServo = new OnboardAngularPositionServo(
                        logger, wristMech, ref, wristFeedback);

                // Gravity gravity = new Gravity(logger, 0, 0);
                // Spring spring = new Spring(logger);
                // m_gravityAndSpringTorque = new Torque((x) -> gravity.applyAsDouble(x) + spring.applyAsDouble(x));
                // for now, no extra forces in simulation
                // TODO: add physics to the sim
                m_gravityAndSpringTorque = new Torque(x->x);
                m_wristMech = wristMech;
            }

        }
        wristServo.reset();
        if (VISUALIZE) {
            m_viz = new WristVisualization(this);
        } else {
            m_viz = () -> {
            };
        }
    }


    @Override
    public void periodic() {
        wristServo.periodic();
        m_log_safe.log(() -> m_isSafe);
        m_viz.run();
    }

    public void resetWristProfile() {
        wristServo.reset();
    }

    public boolean atSetpoint() {
        return wristServo.atSetpoint();
    }

    public boolean profileDone() {
        return wristServo.profileDone();
    }

    public void setWristDutyCycle(double value) {
        m_wristMech.setDutyCycle(value);
    }

    // public void setStatic() {
    // wristServo.setStaticTorque(2.1);
    // }

    public double getAngle() {
        // note this default might be dangerous
        return wristServo.getPosition().orElse(0);
    }

    public void setAngle() {
        double torque = m_gravityAndSpringTorque.torque(wristServo.getPosition());
        wristServo.setPositionProfiled(0.2, torque);
    }

    public void setAngleValue(double goal) {
        double torque = m_gravityAndSpringTorque.torque(wristServo.getPosition());
        wristServo.setPositionProfiled(goal, torque);
    }

    public void setAngleDirect(Setpoints1d setpoint) {
        double torque = m_gravityAndSpringTorque.torque(wristServo.getPosition());
        wristServo.setPositionDirect(setpoint, torque);
    }

    public void setAngleSafe() {
        double torque = m_gravityAndSpringTorque.torque(wristServo.getPosition());
        wristServo.setPositionProfiled(-0.1, torque);
    }

    public boolean getSafeCondition() {
        return m_isSafe;
    }

    public void setSafeCondition(boolean isSafe) {
        m_isSafe = isSafe;
    }

    public void stop() {
        wristServo.stop();
    }

}
