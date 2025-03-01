// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import java.lang.Thread.State;
import java.util.ResourceBundle.Control;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.GravityServoInterface;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardAngularPositionServoWithoutAbsolute;
import org.team100.lib.motion.servo.OutboardAngularPositionServoWithoutWrap;
import org.team100.lib.motion.servo.OutboardGravityServo;
import org.team100.lib.motion.servo.OutboardLinearPositionServo;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.state.Control100;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase implements Glassy {
    /** Creates a new Elevator. */

    private static final double kElevatorReduction = 2; // TODO CHANGE THIS
    private static final double kElevatorWheelDiamater = 1; // TODO CHANGE THIS

    private static final double kWristReduction = 1; // TODO CHANGE THIS
    private static final double kWristWheelDiameter = 1; // TODO CHANGE THIS

    private final GravityServoInterface wristServo;


    public Wrist(
            LoggerFactory parent,
            int wristID) {

        LoggerFactory child = parent.child(this);

        LoggerFactory wristLogger = child.child("Wrist");


        LoggerFactory wristMotorLogger = child.child("Wrist Motor");



        int wristSupplyLimit = 60;
        int wristStatorLimit = 90;

        PIDConstants wristPID = PIDConstants.makePositionPID(10);

        Feedforward100 wristFF = Feedforward100.makeKraken6Wrist();
        PIDController wristPIDController = new PIDController(wristPID.getPositionP(), wristPID.getPositionI(),
                wristPID.getPositionD());
        wristPIDController.setTolerance(0.02);
        wristPIDController.setIntegratorRange(0, 1);
        TrapezoidProfile100 wristProfile = new TrapezoidProfile100(20, 20, 0.05); // TODO CHANGE THESE

        switch (Identity.instance) {
            case FRC_100_ea4 -> {
                
                Kraken6Motor wristMotor = new Kraken6Motor(wristMotorLogger, wristID, MotorPhase.REVERSE, 
                        wristSupplyLimit, wristStatorLimit, wristPID, wristFF);

                

             
                RotaryPositionSensor encoder = new AS5048RotaryPositionSensor(
                        child,
                        9,
                        0.139409,
                        EncoderDrive.DIRECT,
                        false);

                IncrementalBareEncoder internalWristEncoder  = new Talon6Encoder(wristLogger, wristMotor);

                RotaryMechanism wristMech = new SimpleRotaryMechanism(wristLogger, wristMotor, internalWristEncoder, 9);

                CombinedEncoder combinedEncoder = new CombinedEncoder(wristLogger, encoder, wristMech, false);


                AngularPositionServo wristServoWithoutGravity = new OutboardAngularPositionServoWithoutWrap(child, wristMech, combinedEncoder, wristProfile);


                wristServo = new OutboardGravityServo(wristServoWithoutGravity, 4, 0);
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
                wristServo = new OutboardGravityServo(wristServoWithoutGravity, 0, 0);
            }

        }
    }

    @Override
    public void periodic() {
        // This method will be called on    ce per scheduler run

        wristServo.periodic();
    }

    public void resetWristProfile() {
        wristServo.reset();
    }




    public double getAngle() {
        return wristServo.getPositionRad().orElse(0);
    }

    public void setAngle(){
        Control100 control = new Control100(3.985623, 0, 0); //1.17 for l3
        wristServo.setState(control);
    }


    public void setAngleSafe(){
        Control100 control = new Control100(0.551210, 0, 0); //1.17 for l3
        wristServo.setState(control);
    }
}
