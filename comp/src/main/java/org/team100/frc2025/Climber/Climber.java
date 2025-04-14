package org.team100.frc2025.Climber;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.simple.IncrementalProfiledController;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
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
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Model100;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {




    // accepts setpoints
    AngularPositionServo climberMotor;
    // BareMotor m_motor;

    public Climber(LoggerFactory logger, int canID) {
        LoggerFactory child = logger.child("Climber");

        switch (Identity.instance) {
            case COMP_BOT -> {
                Falcon6Motor motor = new Falcon6Motor(child, canID, MotorPhase.REVERSE, 50, 50,
                        PIDConstants.makePositionPID(1),
                        Feedforward100.makeArmPivot());

                Talon6Encoder encoder = new Talon6Encoder(child, motor);

                // this reads the arm angle directly
                RotaryPositionSensor sensor = new AS5048RotaryPositionSensor(
                        child,
                        3,
                        0.110602,
                        EncoderDrive.DIRECT,
                        false);

                double gearRatio = 25 * 3 * 4;
                RotaryMechanism rotaryMechanism = new RotaryMechanism(
                        child, motor, sensor, gearRatio, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                Profile100 profile100 = new TrapezoidProfile100(0.5, 0.5, 0.05);

                PIDFeedback feedback = new PIDFeedback(child, 10, 0, 0, false, 0.05, 0.1);
                // TODO: remove this
                IncrementalProfileReference1d ref = new IncrementalProfileReference1d(
                        profile100, new Model100(1, 0));

                // TODO: remove this
                ProfiledController controller = new IncrementalProfiledController(
                        child, ref, feedback, x -> x, 0.05, 0.05);

                climberMotor = new OnboardAngularPositionServo(child, rotaryMechanism, feedback);
            }

            default -> {
                SimulatedBareMotor climberMotor = new SimulatedBareMotor(child, 100);

                SimulatedBareEncoder encoder = new SimulatedBareEncoder(child, climberMotor);
                SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(child, encoder, 1, () -> 0);

                RotaryMechanism climberMech = new RotaryMechanism(
                        child,
                        climberMotor,
                        sensor,
                        1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                // ProfiledController controller = new TimedProfiledController();

                // climberMotor = new OnboardAngularPositionServo(
                // child, climberMech, encoder, m_controller);

            }

        }

    }

    public void setDutyCycle(double dutyCycle) {
        if (climberMotor == null)
            return;
        climberMotor.setDutyCycle(dutyCycle);
    }

    // public void setAngle(double value) {


    //     climberMotor.setPositionGoal(value, 0);
    // }

    public void setAngleSetpoint(Setpoints1d setpoint) {
        climberMotor.setPositionSetpoint(setpoint, 0);
    }

    public double getAngle() {
        return climberMotor.getPosition().getAsDouble();
    }

    public void reset() {
        climberMotor.reset();
    }

    @Override
    public void periodic() {
        if (climberMotor == null)
            return;
        climberMotor.periodic();
    }
}
