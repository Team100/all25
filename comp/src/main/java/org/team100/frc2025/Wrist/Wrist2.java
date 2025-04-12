package org.team100.frc2025.Wrist;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.FullStateFeedback;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.controller.simple.TimedProfiledController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.GravityServoInterface;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardGravityServo;
import org.team100.lib.motion.servo.OutboardGravityServo.Gravity;
import org.team100.lib.motion.servo.OutboardGravityServo.Spring;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.timed.JerkLimitedProfile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist2 extends SubsystemBase implements Glassy {
    /**
     * Publish the elevator mechanism visualization to glass. This might be a little
     * bit slow, turn it off for comp.
     */
    private static final boolean VISUALIZE = true;
    private static final int GEAR_RATIO = 25;

    private static final double kWristMinimumPosition = -0.5;
    private static final double kWristMaximumPosition = 4;

    // private LaserCan lc;

    private boolean m_isSafe = false;

    private final double kPositionTolerance = 0.02;
    private final double kVelocityTolerance = 0.01;
    private final GravityServoInterface wristServo;

    private final RotaryMechanism m_wristMech;
    private final ProfiledController m_controller;
    private final BooleanLogger safeLogger;
    private final Runnable m_viz;

    public Wrist2(LoggerFactory parent, int wristID) {
        LoggerFactory wristLogger = parent.child(this);

        int wristSupplyLimit = 60;
        int wristStatorLimit = 90;

        int algaeCurrentLimit = 20;
        int coralCurrentLimit = 20;

        PIDConstants wristPID = PIDConstants.makeVelocityPID(0.3); // 0.3

        Feedforward100 wristFF = Feedforward100.makeKraken6Wrist();

        Feedback100 wristFeedback = new FullStateFeedback(wristLogger,
                4.0, 0.11, x -> x, 0.05, 0.05);

        // TrapezoidProfile100 wristProfile = new TrapezoidProfile100(35, 15,
        // kPositionTolerance);

        double maxVel = 40;
        double maxAccel = 40;
        double maxJerk = 70;

        safeLogger = wristLogger.booleanLogger(Level.TRACE, "Wrist Safe Condition");

        switch (Identity.instance) {
            case COMP_BOT -> {

                Kraken6Motor wristMotor = new Kraken6Motor(wristLogger, wristID, MotorPhase.REVERSE,
                        wristSupplyLimit, wristStatorLimit, wristPID, wristFF);

                // this reads the wrist angle directly.
                RotaryPositionSensor sensor = new AS5048RotaryPositionSensor(
                        wristLogger,
                        5,
                        0.135541, // 0.346857, //0.317012, //0.227471, //0.188726
                        EncoderDrive.DIRECT,
                        false);

                m_controller = new TimedProfiledController(parent,
                        new JerkLimitedProfile100(maxVel, maxAccel, maxJerk, false),
                        wristFeedback, x -> x, 0.01, 0.01); // WAS 0.05

                IncrementalBareEncoder internalWristEncoder = new Talon6Encoder(wristLogger, wristMotor);

                m_wristMech = new RotaryMechanism(
                        wristLogger,
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

                // ProfiledController controller = new
                // IncrementalProfiledController(wristProfile, wristFeedback, x -> x,
                // kPositionTolerance, kPositionTolerance);

                AngularPositionServo wristServoWithoutGravity = new OnboardAngularPositionServo(
                        wristLogger, m_wristMech, m_controller);

                wristServoWithoutGravity.reset();

                Gravity gravity = new Gravity(wristLogger, 9.0, -0.451230);
                Spring spring = new Spring(wristLogger);
                wristServo = new OutboardGravityServo(
                        wristLogger, wristServoWithoutGravity,
                        (x) -> gravity.applyAsDouble(x) + spring.applyAsDouble(x));

                m_controller.init(new Model100(sensor.getPositionRad().orElseThrow(), 0));

            }
            default -> {

                SimulatedBareMotor motor = new SimulatedBareMotor(wristLogger, 100);
                SimulatedBareEncoder encoder = new SimulatedBareEncoder(wristLogger, motor);
                SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                        wristLogger, encoder, GEAR_RATIO, () -> 0);
                RotaryMechanism wristMech = new RotaryMechanism(
                        wristLogger, motor, sensor,
                        GEAR_RATIO, kWristMinimumPosition, kWristMaximumPosition);

                JerkLimitedProfile100 profile = new JerkLimitedProfile100(maxVel, maxAccel, maxJerk, false);
                m_controller = new TimedProfiledController(wristLogger,
                        profile, wristFeedback, x -> x, 0.1, 0.05);

                AngularPositionServo wristServoWithoutGravity = new OnboardAngularPositionServo(
                        wristLogger, wristMech, m_controller);

                Gravity gravity = new Gravity(wristLogger, 0, 0);
                Spring spring = new Spring(wristLogger);
                wristServo = new OutboardGravityServo(
                        wristLogger, wristServoWithoutGravity,
                        (x) -> gravity.applyAsDouble(x) + spring.applyAsDouble(x));
                m_wristMech = wristMech;

                m_controller.init(new Model100(sensor.getPositionRad().orElseThrow(), 0));
            }

        }
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
        safeLogger.log(() -> m_isSafe);
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

    public void setStatic() {
        wristServo.setStaticTorque(2.1);
    }

    public double getAngle() {
        return wristServo.getPositionRad().orElse(0);
    }

    public void setAngle() {
        Control100 control = new Control100(0.2, 0, 0); // 1.17 for l3
        wristServo.setState(control);
    }

    public void setAngleValue(double goal) {
        // System.out.println(" I AM BEING CALLED WITH GOAL" + goal);
        Control100 control = new Control100(goal, 0, 0); // 1.17 for l3
        wristServo.setState(control);
    }

    public void setAngleSafe() {
        Control100 control = new Control100(-0.1, 0, 0); // 1.17 for l3
        wristServo.setState(control);
    }

    public boolean getSafeCondition() {
        return m_isSafe;
    }

    public void setSafeCondition(boolean isSafe) {
        // System.out.println("I AM BEING CALLED WITH BOOLEAN " + isSafe);
        m_isSafe = isSafe;
    }

    public void stop() {
        wristServo.stop();
    }

    public void close() {
        m_controller.close();
    }
}
