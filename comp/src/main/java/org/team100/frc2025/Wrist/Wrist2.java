// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.CombinedEncoder;
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
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.GravityServoInterface;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardGravityServo;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.state.Control100;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist2 extends SubsystemBase implements Glassy {
    /** Creates a new Elevator. */

    private LaserCan lc;

    private boolean m_isSafe = false;

    private final double kPositionTolerance = 0.02;
    private final GravityServoInterface wristServo;

    private final RotaryMechanism m_wristMech;
    private final BooleanLogger safeLogger;

    public Wrist2(
            LoggerFactory parent,
            int wristID) {

        LoggerFactory child = parent.child(this);

        LoggerFactory wristLogger = child.child("Wrist 2");

        LoggerFactory wristMotorLogger = child.child("Wrist 2 Motor");
        


        int wristSupplyLimit = 60;
        int wristStatorLimit = 90;

        int algaeCurrentLimit = 20;
        int coralCurrentLimit = 20;

        PIDConstants wristPID = PIDConstants.makeVelocityPID(0.3); //31

        Feedforward100 wristFF = Feedforward100.makeKraken6Wrist();

        TrapezoidProfile100 wristProfile = new TrapezoidProfile100(35, 15, kPositionTolerance); // TODO CHANGE THESE

        safeLogger = parent.booleanLogger(Level.TRACE, "Wrist Safe Condition");

        switch (Identity.instance) {
            case COMP_BOT -> {
                
                Kraken6Motor wristMotor = new Kraken6Motor(wristMotorLogger, wristID, MotorPhase.REVERSE, 
                        wristSupplyLimit, wristStatorLimit, wristPID, wristFF);

                
             
                RotaryPositionSensor encoder = new AS5048RotaryPositionSensor(
                        child,
                        5,
                        0.188726,
                        EncoderDrive.DIRECT,
                        false);
 
                IncrementalBareEncoder internalWristEncoder  = new Talon6Encoder(wristLogger, wristMotor);

                RotaryMechanism wristMech = new SimpleRotaryMechanism(wristLogger, wristMotor, internalWristEncoder, 25);

                m_wristMech = wristMech;

                Feedback100 wristFeedback = new PIDFeedback(parent, 7.5, 0.00, 0.000 , false, kPositionTolerance, kPositionTolerance); 
                // Feedback100 wristFeedback = new PIDFeedback(parent, 0, 0, 0 , false, kPositionTolerance, kPositionTolerance); 

                ProfiledController controller = new ProfiledController(wristProfile, wristFeedback, x -> x, kPositionTolerance, kPositionTolerance);

                AngularPositionServo wristServoWithoutGravity = new OnboardAngularPositionServo(child, wristMech, encoder, controller);
                wristServoWithoutGravity.reset();

                wristServo = new OutboardGravityServo(child, wristServoWithoutGravity, 0, 0); //
                
                

                break;
            }
            default -> {
                

               
                SimulatedBareMotor wristMotor = new SimulatedBareMotor(wristLogger, 100);
                RotaryMechanism wristMech = new SimpleRotaryMechanism(wristLogger, wristMotor,
                        new SimulatedBareEncoder(wristLogger, wristMotor), 10.5);
                SimulatedRotaryPositionSensor encoder = new SimulatedRotaryPositionSensor(wristLogger, wristMech);
                CombinedEncoder combinedEncoder = new CombinedEncoder(wristLogger, encoder, wristMech, false);
                AngularPositionServo wristServoWithoutGravity = new OutboardAngularPositionServo(child, wristMech,
                        combinedEncoder, wristProfile);
                wristServo = new OutboardGravityServo(child, wristServoWithoutGravity, 0, 0);
                m_wristMech = wristMech;

                // m_algaeMech = Neo550Factory.getNEO550LinearMechanism(getName(), child, algaeCurrentLimit, algaeID, 1, MotorPhase.FORWARD, 1);
            }

        }
    }

    @Override
    public void periodic() {
        wristServo.periodic();
        safeLogger.log(() -> m_isSafe);
        
    }

    public void resetWristProfile() {
        wristServo.reset();
    }

    public boolean atSetpoint() {
        return wristServo.atSetpoint();
    }

    public void setWristDutyCycle(double value){
        m_wristMech.setDutyCycle(value);
    }

    public void setStatic(){
        wristServo.setStaticTorque(2.1);
    }

    public double getAngle() {
        return wristServo.getPositionRad().orElse(0);
    }

    public void setAngle(){
        Control100 control = new Control100(0.2, 0, 0); //1.17 for l3
        wristServo.setState(control);
    }

    public void setAngleValue(double value){
        Control100 control = new Control100(value, 0, 0); //1.17 for l3
        wristServo.setState(control);
    }


    public void setAngleSafe(){
        Control100 control = new Control100(-0.1, 0, 0); //1.17 for l3
        wristServo.setState(control);
    }

    public boolean getSafeCondition(){
        return m_isSafe;
    }

    public void setSafeCondition(boolean isSafe){
        m_isSafe = isSafe;
    }

    public void stop(){
        wristServo.stop();
    }   

}
