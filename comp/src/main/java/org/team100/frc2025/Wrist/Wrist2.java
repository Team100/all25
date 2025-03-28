package org.team100.frc2025.Wrist;

import org.team100.lib.async.Async;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.controller.simple.SelectProfiledController;
import org.team100.lib.controller.simple.SelectProfiledController.ProfileChoice;
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
import org.team100.lib.motion.mechanism.LimitedRotaryMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.GravityServoInterface;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardGravityServo;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.PolledEnumChooser;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist2 extends SubsystemBase implements Glassy {
    private static final double kWristMinimumPosition = -0.5;
    private static final double kWristMaximumPosition = 4;

    private LaserCan lc;

    private boolean m_isSafe = false;

    private final double kPositionTolerance = 0.02;
    private final double kVelocityTolerance = 0.01;
    private final GravityServoInterface wristServo;

    private final RotaryMechanism m_wristMech;
    private final SelectProfiledController m_controller;
    private final BooleanLogger safeLogger;
    private final WristVisualization m_viz;
    private final PolledEnumChooser<ProfileChoice> m_profileChooser;

    public Wrist2(
            LoggerFactory parent,
            Async async,
            int wristID) {

        LoggerFactory child = parent.child(this);

        LoggerFactory wristLogger = child.child("Wrist 2");

        LoggerFactory wristMotorLogger = child.child("Wrist 2 Motor");

        int wristSupplyLimit = 60;
        int wristStatorLimit = 90;

        int algaeCurrentLimit = 20;
        int coralCurrentLimit = 20;

        PIDConstants wristPID = PIDConstants.makeVelocityPID(0.1); // 0.11

        Feedforward100 wristFF = Feedforward100.makeKraken6Wrist();
        Feedback100 wristFeedback = new PIDFeedback(child,
                5.0, 0.0, 0.0, false,
                kPositionTolerance, 0.1);

        // TrapezoidProfile100 wristProfile = new TrapezoidProfile100(35, 15,
        // kPositionTolerance);

        double maxVel = 35;
        double maxAccel = 5;
        double stallAccel = 60;
        double maxJerk = 20;

        // ProfiledController controller = makeProfiledController(
        // wristLogger,
        // wristFeedback,
        // maxVel,
        // maxAccel,
        // stallAccel,
        // maxJerk,
        // kPositionTolerance,
        // kVelocityTolerance);

        // TrapezoidProfile100 wristProfile = new TrapezoidProfile100(35, 5,
        // kPositionTolerance); // TODO CHANGE THESE
        // TrapezoidProfile100 wristFastProfile = new TrapezoidProfile100(35, 5,
        // kPositionTolerance); // TODO CHANGE THESE
        // TrapezoidProfile100 wristSlowProfile = new TrapezoidProfile100(35, 5,
        // kPositionTolerance); // TODO CHANGE THESE
        // DualProfile wristDualProfile = new DualProfile(wristFastProfile,
        // wristSlowProfile, 0.2);

        safeLogger = child.booleanLogger(Level.TRACE, "Wrist Safe Condition");

        m_profileChooser = new PolledEnumChooser<>(
                async,
                ProfileChoice.class,
                "wrist profile",
                ProfileChoice.TRAPEZOID_100);

        switch (Identity.instance) {
            case COMP_BOT -> {

                Kraken6Motor wristMotor = new Kraken6Motor(wristMotorLogger, wristID, MotorPhase.REVERSE,
                        wristSupplyLimit, wristStatorLimit, wristPID, wristFF);

                RotaryPositionSensor encoder = new AS5048RotaryPositionSensor(
                        child,
                        5,
                        0.135541, // 0.346857, //0.317012, //0.227471, //0.188726
                        EncoderDrive.DIRECT,
                        false);

                m_controller = new SelectProfiledController(
                        child,
                        wristFeedback,
                        x -> x,
                        () -> encoder.getPositionRad().orElseThrow(),
                        maxVel,
                        maxAccel,
                        stallAccel,
                        maxJerk,
                        kPositionTolerance,
                        kVelocityTolerance);
                m_profileChooser.register(m_controller::setDelegate);

                IncrementalBareEncoder internalWristEncoder = new Talon6Encoder(wristLogger, wristMotor);

                RotaryMechanism wristMech = new SimpleRotaryMechanism(wristLogger, wristMotor, internalWristEncoder,
                        25);

                // TODO: what are the correct limits?
                RotaryMechanism limitedMech = new LimitedRotaryMechanism(wristMech, kWristMinimumPosition, 4);

                m_wristMech = wristMech;

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

                AngularPositionServo wristServoWithoutGravity = new OnboardAngularPositionServo(child, limitedMech,
                        encoder, m_controller);
                wristServoWithoutGravity.reset();
                wristServo = new OutboardGravityServo(child, wristServoWithoutGravity, 9.5, -0.366925);
                m_controller.init(new Model100(encoder.getPositionRad().orElseThrow(), 0));

            }
            default -> {

                SimulatedBareMotor wristMotor = new SimulatedBareMotor(wristLogger, 100);
                RotaryMechanism wristMech = new SimpleRotaryMechanism(wristLogger, wristMotor,
                        new SimulatedBareEncoder(wristLogger, wristMotor), 10.5);

                RotaryMechanism limitedMech = new LimitedRotaryMechanism(wristMech, kWristMinimumPosition,
                        kWristMaximumPosition);

                SimulatedRotaryPositionSensor encoder = new SimulatedRotaryPositionSensor(wristLogger, wristMech,
                        () -> 0);
                m_controller = new SelectProfiledController(
                        child,
                        wristFeedback,
                        x -> x,
                        () -> encoder.getPositionRad().orElseThrow(),
                        maxVel,
                        maxAccel,
                        stallAccel,
                        maxJerk,
                        kPositionTolerance,
                        kVelocityTolerance);
                m_profileChooser.register(m_controller::setDelegate);

                // CombinedEncoder combinedEncoder = new CombinedEncoder(wristLogger, encoder,
                // wristMech, false);
                AngularPositionServo wristServoWithoutGravity = new OnboardAngularPositionServo(child, wristMech,
                        encoder, m_controller);
                wristServo = new OutboardGravityServo(child, wristServoWithoutGravity, 0, 0);
                m_wristMech = limitedMech;

                m_controller.init(new Model100(encoder.getPositionRad().orElseThrow(), 0));
                // m_algaeMech = Neo550Factory.getNEO550LinearMechanism(getName(), child,
                // algaeCurrentLimit, algaeID, 1, MotorPhase.FORWARD, 1);
            }

        }
        m_viz = new WristVisualization(this);
    }

    @Override
    public void periodic() {
        wristServo.periodic();
        safeLogger.log(() -> m_isSafe);
        m_viz.viz();
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
        m_isSafe = isSafe;
    }

    public void stop() {
        wristServo.stop();
    }

    public void close() {
        m_controller.close();
        m_profileChooser.close();
    }
}
