package org.team100.frc2025.Climber;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final AngularPositionServo m_servo;

    public Climber(LoggerFactory parent, int canID) {
        LoggerFactory log = parent.child("Climber");

        Profile100 profile100 = new TrapezoidProfile100(0.5, 0.5, 0.05);
        ProfileReference1d ref = new IncrementalProfileReference1d(profile100, 0.05, 0.05);
        PIDFeedback feedback = new PIDFeedback(log, 10, 0, 0, false, 0.05, 0.1);

        switch (Identity.instance) {
            case COMP_BOT -> {
                Falcon6Motor motor = new Falcon6Motor(log, canID, MotorPhase.REVERSE, 50, 50,
                        PIDConstants.makePositionPID(1),
                        Feedforward100.makeArmPivot());

                int channel = 3;
                double inputOffset = 0.110602;
                RotaryPositionSensor sensor = new AS5048RotaryPositionSensor(
                        log, channel, inputOffset, EncoderDrive.DIRECT);

                double gearRatio = 25 * 3 * 4;
                RotaryMechanism rotaryMechanism = new RotaryMechanism(
                        log, motor, sensor, gearRatio, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                m_servo = new OnboardAngularPositionServo(log, rotaryMechanism, ref, feedback);
            }

            default -> {
                SimulatedBareMotor climberMotor = new SimulatedBareMotor(log, 100);

                SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, climberMotor);
                SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1);

                RotaryMechanism climberMech = new RotaryMechanism(
                        log,
                        climberMotor,
                        sensor,
                        1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_servo = new OnboardAngularPositionServo(log, climberMech, ref, feedback);
            }

        }

    }

    public void setDutyCycle(double dutyCycle) {
        if (m_servo == null)
            return;
        m_servo.setDutyCycle(dutyCycle);
    }

    public void setAngle(double value) {
        m_servo.setPositionProfiled(value, 0);
    }

    public void setAngleSetpoint(Setpoints1d setpoint) {
        m_servo.setPositionDirect(setpoint, 0);
    }

    public double getAngle() {
        return m_servo.getPosition().getAsDouble();
    }

    public void reset() {
        m_servo.reset();
    }

    @Override
    public void periodic() {
        if (m_servo == null)
            return;
        m_servo.periodic();
    }
}
