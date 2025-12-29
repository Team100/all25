package org.team100.frc2025.Climber;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.ctre.Falcon6Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.sensor.position.absolute.RotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.wpi.AS5048RotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OnboardAngularPositionServo;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final AngularPositionServo m_servo;

    public Climber(LoggerFactory parent, CanId canID) {
        LoggerFactory log = parent.type(this);

        IncrementalProfile profile = new TrapezoidIncrementalProfile(log, 1, 2, 0.05);
        ProfileReferenceR1 ref = new IncrementalProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        PIDFeedback feedback = new PIDFeedback(log, 5, 0, 0, false, 0.05, 0.1);

        switch (Identity.instance) {
            case COMP_BOT -> {
                Falcon6Motor motor = new Falcon6Motor(log, canID, NeutralMode.BRAKE, MotorPhase.REVERSE,
                        20, 20,
                        PIDConstants.makePositionPID(log, 1),
                        Feedforward100.makeArmPivot(log));

                double inputOffset = 0.440602;
                RotaryPositionSensor sensor = new AS5048RotaryPositionSensor(
                        log, new RoboRioChannel(0), inputOffset, EncoderDrive.DIRECT);
                double gearRatio = 5 * 5 * 4 * 20;

                RotaryMechanism rotaryMechanism = new RotaryMechanism(
                        log, motor, sensor, gearRatio,
                        0, Math.PI / 2);

                m_servo = new OnboardAngularPositionServo(log, rotaryMechanism, ref, feedback);
            }

            default -> {
                SimulatedBareMotor climberMotor = new SimulatedBareMotor(log, 600);

                IncrementalBareEncoder encoder = climberMotor.encoder();
                SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1);

                RotaryMechanism climberMech = new RotaryMechanism(
                        log, climberMotor, sensor, 1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                m_servo = new OnboardAngularPositionServo(log, climberMech, ref, feedback);
            }
        }
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

    public boolean atGoal() {
        return m_servo.atGoal();
    }

    public double angle() {
        return m_servo.getWrappedPositionRad();
    }

    public void stopMotor() {
        m_servo.stop();
    }

    // COMMANDS

    public Command stop() {
        return run(
                () -> setDutyCycle(0));
    }

    public Command manual(DoubleSupplier s) {
        return runEnd(
                () -> setDutyCycle(s.getAsDouble()),
                () -> setDutyCycle(0));
    }

    /** Push the climber out into the intake position. */
    public Command goToIntakePosition() {
        return startRun(
                () -> reset(),
                () -> setAngle(Math.PI / 2));
    }

    /** Pull the climber in all the way to the climb position. */
    public Command goToClimbPosition() {
        return startRun(
                () -> reset(),
                () -> setAngle(0));
    }

    ////////////////////////////////

    private void reset() {
        m_servo.reset();
    }

    private void setDutyCycle(double dutyCycle) {
        m_servo.setDutyCycle(dutyCycle);
    }

    /** Use a profile to set the position. */
    private void setAngle(double value) {
        m_servo.setPositionProfiled(value, 0);
    }
}
