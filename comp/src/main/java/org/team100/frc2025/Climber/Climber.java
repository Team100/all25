package org.team100.frc2025.Climber;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.simple.IncrementalProfiledController;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.controller.simple.TimedProfiledController;
import org.team100.lib.controller.simple.ZeroFeedback;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardAngularPositionServoWithoutAbsolute;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    AngularPositionServo climberMotor;
    // BareMotor m_motor;

    public Climber(LoggerFactory logger, int canID) {
        LoggerFactory child = logger.child("Climber");

        switch(Identity.instance){
            case COMP_BOT -> {
                Falcon6Motor motor = new Falcon6Motor(child, canID, MotorPhase.REVERSE, 50, 50, PIDConstants.makePositionPID(1),
                    Feedforward100.makeArmPivot());
    
                RotaryMechanism rotaryMechanism = new SimpleRotaryMechanism(child, motor, new Talon6Encoder(child, motor),
                    25 * 3 * 4);
    
                Profile100 profile100 = new TrapezoidProfile100(30, 30, 0.05);
                PIDFeedback feedback = new PIDFeedback(child, 10, 0, 0, false, 0.05, 0.1 );
    
                ProfiledController controller = new IncrementalProfiledController(
                    child, profile100, feedback, x -> x, 0.05, 0.05);
    
                RotaryPositionSensor encoder = new AS5048RotaryPositionSensor(
                            child,
                            3,
                            0.110602,   
                            EncoderDrive.DIRECT,
                            false);
    
                climberMotor = new OnboardAngularPositionServo(child, rotaryMechanism, encoder, controller);
            }

            default -> {
                SimulatedBareMotor climberMotor = new SimulatedBareMotor(child, 100);

                SimulatedBareEncoder encoder0 = new SimulatedBareEncoder(child, climberMotor);

                RotaryMechanism climberMech = new SimpleRotaryMechanism(
                        child,
                        climberMotor,
                        encoder0,
                        1);

                SimulatedRotaryPositionSensor encoder = new SimulatedRotaryPositionSensor(child, climberMech,
                        () -> 0);

                // ProfiledController controller = new TimedProfiledController();

                // climberMotor = new OnboardAngularPositionServo(
                //             child, climberMech, encoder, m_controller);

                        
            }
            
        }
        
    }

    public void setDutyCycle(double dutyCycle) {
        if(climberMotor == null) return;
        climberMotor.setDutyCycle(dutyCycle);
    }

    public void setAngle(double value) {
        climberMotor.setPosition(value, 0);
    }

    public double getAngle() {
        return climberMotor.getPosition().getAsDouble();
    }

    public void reset() {
        climberMotor.reset();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(climberMotor == null) return;
        climberMotor.periodic();
    }
}
