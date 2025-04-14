package org.team100.frc2025.Elevator;

import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.simple.IncrementalProfiledController;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.controller.simple.ZeroFeedback;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.servo.OutboardLinearPositionServo;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements Glassy {
    /**
     * Publish the elevator mechanism visualization to glass. This might be a little
     * bit slow, turn it off for comp.
     */
    private static final boolean VISUALIZE = true;
    private static final double kElevatorMaximumPosition = 56.0;
    private static final double kElevatorMinimumPosition = 0.0;
    private static final double kElevatorReduction = 2;
    private static final double kElevatorWheelDiamater = 1;
    private final double kPositionTolerance = 0.02;
    private final double kVelocityTolerance = 0.01;

    private final OutboardLinearPositionServo starboardServo;
    private final OutboardLinearPositionServo portServo;

    private final ProfiledController m_starboardController;
    private final ProfiledController m_portController;

    private final Runnable m_viz;

    private boolean m_isSafe = false;

    private ScoringPosition m_targetPosition = ScoringPosition.NONE;

    private InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap(); // elevator height, elevator cg

    public Elevator(
            LoggerFactory parent,
            int starboardID,
            int portID) {

        LoggerFactory child = parent.child(this);

        LoggerFactory starboardLogger = child.child("Starboard");
        LoggerFactory portLogger = child.child("Port");

        LoggerFactory starboardMotorLogger = child.child("Starboard Motor");
        LoggerFactory portMotorLogger = child.child("Port Motor");

        int elevatorSupplyLimit = 60;
        int elevatorStatorLimit = 90;

        // TODO: review these constants
        final double maxVel = 220; // 220
        final double maxAccel = 200; // 240
        final double stallAccel = 1000;
        final double maxJerk = 500;

        PIDConstants elevatorPID = PIDConstants.makePositionPID(2.5); // 6.7

        Feedforward100 elevatorFF = Feedforward100.makeKraken6Elevator();
        // TrapezoidProfile100 elevatorProfile = new TrapezoidProfile100(220, 220,
        // 0.05); // TODO CHANGE THESE
        // TrapezoidProfile100 elevatorProfile = new TrapezoidProfile100(200, 200,
        // 0.05); // TODO CHANGE THESE
        // TrapezoidProfile100 elevatorProfile = new TrapezoidProfile100(150, 150,
        // 0.05); // TODO CHANGE THESE
        TrapezoidProfile100 elevatorProfile = new TrapezoidProfile100(190, 210, 0.01); // TODO CHANGE THESE
        // TrapezoidProfile100 elevatorProfile = new TrapezoidProfile100(200, 200,
        // 0.05); // TODO CHANGE THESE
        // TrapezoidProfile100 elevatorProfile = new TrapezoidProfile100(150, 150,
        // 0.05); // TODO CHANGE THESE
        // TrapezoidProfile100 elevatorProfile = new TrapezoidProfile100(130, 100,
        // 0.01); // TODO CHANGE THESE

        table.put(0.0, 0.5);
        table.put(0.0, 0.5);

        Profile100 profile = new TrapezoidProfile100(maxVel, maxAccel, kPositionTolerance);
        // TODO: goal here is wrong
        IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, new Model100(0, 0));
        switch (Identity.instance) {
            case COMP_BOT -> {
                Kraken6Motor starboardMotor = new Kraken6Motor(starboardMotorLogger, starboardID, MotorPhase.REVERSE,
                        elevatorSupplyLimit, elevatorStatorLimit, elevatorPID, elevatorFF);
                Kraken6Motor portMotor = new Kraken6Motor(portMotorLogger, portID, MotorPhase.FORWARD,
                        elevatorSupplyLimit, elevatorStatorLimit, elevatorPID, elevatorFF);

                Talon6Encoder stbdEncoder = new Talon6Encoder(starboardLogger, starboardMotor);

                // TODO: the port and stbd profiles should be the same, i.e. a single chooser
                // for both.
                m_starboardController = new IncrementalProfiledController(
                        child,
                        ref,
                        new ZeroFeedback(x -> x, kPositionTolerance, kVelocityTolerance),
                        x -> x,
                        kPositionTolerance,
                        kVelocityTolerance);

                LinearMechanism starboardMech = new LinearMechanism(
                        starboardMotor,
                        stbdEncoder,
                        kElevatorReduction,
                        kElevatorWheelDiamater,
                        kElevatorMinimumPosition,
                        kElevatorMaximumPosition);

                starboardServo = new OutboardLinearPositionServo(
                        starboardLogger,
                        starboardMech,
                        m_starboardController);

                Talon6Encoder portEncoder = new Talon6Encoder(portLogger, portMotor);

                m_portController = new IncrementalProfiledController(
                        child,
                        ref,
                        new ZeroFeedback(x -> x, kPositionTolerance, kVelocityTolerance),
                        x -> x,
                        kPositionTolerance,
                        kVelocityTolerance);

                LinearMechanism portMech = new LinearMechanism(
                        portMotor,
                        portEncoder,
                        kElevatorReduction,
                        kElevatorWheelDiamater,
                        kElevatorMinimumPosition,
                        kElevatorMaximumPosition);

                portServo = new OutboardLinearPositionServo(
                        portLogger,
                        portMech,
                        m_portController);

            }
            default -> {
                SimulatedBareMotor starboardMotor = new SimulatedBareMotor(starboardMotorLogger, 100);
                SimulatedBareMotor portMotor = new SimulatedBareMotor(portMotorLogger, 100);

                SimulatedBareEncoder stbdEncoder = new SimulatedBareEncoder(starboardLogger, starboardMotor);

                m_starboardController = new IncrementalProfiledController(
                        child,
                        ref,
                        new ZeroFeedback(x -> x, kPositionTolerance, kVelocityTolerance),
                        x -> x,
                        kPositionTolerance,
                        kVelocityTolerance);

                LinearMechanism starboardMech = new LinearMechanism(
                        starboardMotor,
                        stbdEncoder,
                        kElevatorReduction,
                        kElevatorWheelDiamater,
                        kElevatorMinimumPosition,
                        kElevatorMaximumPosition);

                SimulatedBareEncoder portEncoder = new SimulatedBareEncoder(portLogger, portMotor);

                m_portController = new IncrementalProfiledController(
                        child,
                        ref,
                        new ZeroFeedback(x -> x, kPositionTolerance, kVelocityTolerance),
                        x -> x,
                        kPositionTolerance,
                        kVelocityTolerance);

                LinearMechanism portMech = new LinearMechanism(
                        portMotor,
                        portEncoder,
                        kElevatorReduction,
                        kElevatorWheelDiamater,
                        kElevatorMinimumPosition,
                        kElevatorMaximumPosition);

                starboardServo = new OutboardLinearPositionServo(
                        starboardLogger,
                        starboardMech,
                        m_starboardController);
                portServo = new OutboardLinearPositionServo(
                        portLogger,
                        portMech,
                        m_portController);
            }

        }
        if (VISUALIZE) {
            m_viz = new ElevatorVisualization(this);
        } else {
            m_viz = () -> {
            };
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        starboardServo.periodic();
        portServo.periodic();
        m_viz.run();
    }

    public void resetElevatorProfile() {
        starboardServo.reset();
        portServo.reset();
    }

    public void setPosition(double x) {
        starboardServo.setPosition(x, 1.3); // 54 max
        portServo.setPosition(x, 1.3); // 54 max
    }

    public boolean profileDone() {
        return starboardServo.profileDone();
    }

    public void setPositionDirectly(double x) {
        // starboardServo.setPositionDirectly(x, 1.3); // 54 max
        // portServo.setPositionDirectly(x, 1.3); // 54 max

        starboardServo.setPosition(x, 1.3); // 54 max
        portServo.setPosition(x, 1.3); // 54 max

    }

    public void setPositionNoGravity(double x) {
        starboardServo.setPosition(x, 0); // 54 max
        portServo.setPosition(x, 0); // 54 max
    }

    public void setStatic() {
        starboardServo.setPosition(starboardServo.getPosition().getAsDouble(), 1.3); // 54 max
        portServo.setPosition(portServo.getPosition().getAsDouble(), 1.3); // 54 max
    }

    public void setDutyCycle(double value) {
        starboardServo.setDutyCycle(value);
        portServo.setDutyCycle(value);
    }

    /**
     */
    public double getPosition() {
        return starboardServo.getPosition().orElse(0);
    }

    public void stop() {
        starboardServo.stop();
        portServo.stop();
    }

    public double getElevatorCG() {
        return table.get(getPosition());
    }

    // DUMB Getters and Setters
    public boolean getSafeCondition() {
        return m_isSafe;
    }

    public void setSafeCondition(boolean isSafe) {
        m_isSafe = isSafe;
    }

    public void setTargetScoringPosition(ScoringPosition position) {
        m_targetPosition = position;
    }

    public ScoringPosition getScoringPosition() {
        return m_targetPosition;
    }

    public void close() {
        m_portController.close();
        m_starboardController.close();
    }

}
