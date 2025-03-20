// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package org.team100.frc2025.Wrist;

// import org.team100.lib.config.Feedforward100;
// import org.team100.lib.config.Identity;
// import org.team100.lib.config.PIDConstants;
// import org.team100.lib.dashboard.Glassy;
// import org.team100.lib.encoder.AS5048RotaryPositionSensor;
// import org.team100.lib.encoder.CombinedEncoder;
// import org.team100.lib.encoder.EncoderDrive;
// import org.team100.lib.encoder.IncrementalBareEncoder;
// import org.team100.lib.encoder.RotaryPositionSensor;
// import org.team100.lib.encoder.SimulatedBareEncoder;
// import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
// import org.team100.lib.encoder.Talon6Encoder;
// import org.team100.lib.logging.LoggerFactory;
// import org.team100.lib.motion.mechanism.LinearMechanism;
// import org.team100.lib.motion.mechanism.RotaryMechanism;
// import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
// import org.team100.lib.motion.servo.AngularPositionServo;
// import org.team100.lib.motion.servo.GravityServoInterface;
// import org.team100.lib.motion.servo.OutboardAngularPositionServo;
// import org.team100.lib.motion.servo.OutboardAngularPositionServoWithoutWrap;
// import org.team100.lib.motion.servo.OutboardGravityServo;
// import org.team100.lib.motor.Kraken6Motor;
// import org.team100.lib.motor.MotorPhase;
// import org.team100.lib.motor.Neo550Factory;
// import org.team100.lib.motor.SimulatedBareMotor;
// import org.team100.lib.profile.TrapezoidProfile100;
// import org.team100.lib.state.Control100;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Wrist extends SubsystemBase implements Glassy {



//     private final GravityServoInterface wristServo;
//     // private final LinearMechanism m_algaeMech;
//     private final LinearMechanism m_coralMech;
//     private final RotaryMechanism m_wristMech;

//     public Wrist(
//             LoggerFactory parent,
//             int wristID,
//             int algaeID,
//             int coralID) {

//         LoggerFactory child = parent.child(this);

//         LoggerFactory wristLogger = child.child("Wrist");

//         LoggerFactory wristMotorLogger = child.child("Wrist Motor");


//         int wristSupplyLimit = 60;
//         int wristStatorLimit = 90;

//         int algaeCurrentLimit = 20;
//         int coralCurrentLimit = 20;

//         PIDConstants wristPID = PIDConstants.makePositionPID(57); //31

//         Feedforward100 wristFF = Feedforward100.makeKraken6Wrist();

//         TrapezoidProfile100 wristProfile = new TrapezoidProfile100(50, 30, 0.05); // TODO CHANGE THESE

//         switch (Identity.instance) {
//             case COMP_BOT -> {
                
//                 Kraken6Motor wristMotor = new Kraken6Motor(wristMotorLogger, wristID, MotorPhase.REVERSE, 
//                         wristSupplyLimit, wristStatorLimit, wristPID, wristFF);

                

             
//                 RotaryPositionSensor encoder = new AS5048RotaryPositionSensor(
//                         child,
//                         5,
//                         0.228270,
//                         EncoderDrive.DIRECT,
//                         false);

//                 IncrementalBareEncoder internalWristEncoder  = new Talon6Encoder(wristLogger, wristMotor);

//                 RotaryMechanism wristMech = new SimpleRotaryMechanism(wristLogger, wristMotor, internalWristEncoder, 9);

//                 m_wristMech = wristMech;

//                 CombinedEncoder combinedEncoder = new CombinedEncoder(wristLogger, encoder, wristMech, false);


//                 AngularPositionServo wristServoWithoutGravity = new OutboardAngularPositionServoWithoutWrap(child, wristMech, combinedEncoder, wristProfile);


//                 wristServo = new OutboardGravityServo(wristServoWithoutGravity, 2, 0); //2

//                 // m_algaeMech = Neo550Factory.getNEO550LinearMechanism(getName(), child, algaeCurrentLimit, algaeID, 1, MotorPhase.REVERSE, 1);
//                 m_coralMech = Neo550Factory.getNEO550LinearMechanism(getName(), child, coralCurrentLimit, coralID, 1, MotorPhase.FORWARD, 1);

//                 break;
//             }
//             default -> {
                

               
//                 SimulatedBareMotor wristMotor = new SimulatedBareMotor(wristLogger, 100);
//                 RotaryMechanism wristMech = new SimpleRotaryMechanism(wristLogger, wristMotor,
//                         new SimulatedBareEncoder(wristLogger, wristMotor), 10.5);
//                 SimulatedRotaryPositionSensor encoder = new SimulatedRotaryPositionSensor(wristLogger, wristMech);
//                 CombinedEncoder combinedEncoder = new CombinedEncoder(wristLogger, encoder, wristMech, false);
//                 AngularPositionServo wristServoWithoutGravity = new OutboardAngularPositionServo(child, wristMech,
//                         combinedEncoder, wristProfile);
//                 wristServo = new OutboardGravityServo(wristServoWithoutGravity, 0, 0);
                
//                 m_coralMech = Neo550Factory.getNEO550LinearMechanism(getName(), child, coralCurrentLimit, coralID, 1, MotorPhase.REVERSE, 1);
//                 m_wristMech = null;
//                 // m_algaeMech = Neo550Factory.getNEO550LinearMechanism(getName(), child, algaeCurrentLimit, algaeID, 1, MotorPhase.FORWARD, 1);
//             }

//         }
//     }

//     @Override
//     public void periodic() {
//         wristServo.periodic();
//     }

//     public void resetWristProfile() {
//         wristServo.reset();
//     }

//     public boolean atSetpoint() {
//         return wristServo.atSetpoint();
//     }

//     public void setWristDutyCycle(double value){
//         m_wristMech.setDutyCycle(value);
//     }




//     public double getAngle() {
//         return wristServo.getPositionRad().orElse(0);
//     }

//     public void setAngle(){
//         Control100 control = new Control100(0.1, 0, 0); //1.17 for l3
//         wristServo.setState(control);
//     }

//     public void setAngleValue(double value){
//         Control100 control = new Control100(value, 0, 0); //1.17 for l3
//         wristServo.setState(control);
//     }


//     public void setAngleSafe(){
//         Control100 control = new Control100(-0.1, 0, 0); //1.17 for l3
//         wristServo.setState(control);
//     }

//     public void setAlgaeMotor(double value){
//         // m_algaeMech.setDutyCycle(value);
//     }

//     public void setCoralMotor(double value){
//         m_coralMech.setDutyCycle(value);
//     }
// }
