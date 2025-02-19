// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Elevator;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.GravityServoInterface;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardGravityServo;
import org.team100.lib.motion.servo.OutboardLinearPositionServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.profile.TrapezoidProfile100;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements Glassy {
  /** Creates a new Elevator. */

  private static final double kElevatorReduction = 1; //TODO CHANGE THIS
  private static final double kElevatorWheelDiamater = 1; //TODO CHANGE THIS

  private static final double kWristReduction = 1; //TODO CHANGE THIS
  private static final double kWristWheelDiameter = 1; //TODO CHANGE THIS


  private final OutboardLinearPositionServo starboardServo;
  private final OutboardLinearPositionServo portServo;
  private final GravityServoInterface wristServo;

  public Elevator(
      LoggerFactory parent,
      int starboardID,
      int portID,
      int wristID) {

	LoggerFactory child = parent.child(this);	

	LoggerFactory starboardLogger = child.child("Starboard");
    LoggerFactory portLogger = child.child("Port");
    LoggerFactory wristLogger = child.child("Wrist");

	int elevatorSupplyLimit = 10;
	int elevatorStatorLimit = 10;

	PIDConstants elevatorPID = PIDConstants.makePositionPID(0.3);
	Feedforward100 elevatorFF = Feedforward100.makeKraken6Elevator();
    TrapezoidProfile100 elevatorProfile = new TrapezoidProfile100(1, 1, 1); //TODO CHANGE THESE

	int wristSupplyLimit = 10;
	int wristStatorLimit = 10;

	PIDConstants wristPID =  PIDConstants.makePositionPID(0.3);
	Feedforward100 wristFF = Feedforward100.makeKraken6Elevator();
    PIDController wristPIDController = new PIDController(wristPID.getPositionP(), wristPID.getPositionI(), wristPID.getPositionD());
    wristPIDController.setTolerance(0.02);
    wristPIDController.setIntegratorRange(0, 1);
    TrapezoidProfile100 wristProfile = new TrapezoidProfile100(1, 1, 1); //TODO CHANGE THESE


	switch(Identity.instance) {
		case COMP_BOT:
			Kraken6Motor starboardMotor = new Kraken6Motor(parent, starboardID, MotorPhase.FORWARD, elevatorSupplyLimit, elevatorStatorLimit, elevatorPID, elevatorFF);
			Kraken6Motor portMotor = new Kraken6Motor(parent, portID, MotorPhase.REVERSE, elevatorSupplyLimit, elevatorStatorLimit, elevatorPID, elevatorFF);
			Kraken6Motor wristMotor = new Kraken6Motor(parent, wristID, MotorPhase.FORWARD, wristSupplyLimit, wristStatorLimit, wristPID, wristFF);

			LinearMechanism starboardMech = new SimpleLinearMechanism(
						starboardMotor,
                        new Talon6Encoder(starboardLogger, starboardMotor),
                        kElevatorReduction,
                        kElevatorWheelDiamater);
            
            starboardServo = new OutboardLinearPositionServo(starboardLogger, starboardMech, elevatorProfile);
            
            LinearMechanism portMech = new SimpleLinearMechanism(
						portMotor,
                        new Talon6Encoder(portLogger, portMotor),
                        kElevatorReduction,
                        kElevatorWheelDiamater);
            
            portServo = new OutboardLinearPositionServo(portLogger, portMech, elevatorProfile);

            RotaryMechanism wristMech = new SimpleRotaryMechanism(
                        wristLogger,
						wristMotor,
                        new Talon6Encoder(portLogger, portMotor),
                        kElevatorReduction);

            AS5048RotaryPositionSensor encoder = new AS5048RotaryPositionSensor(
                        wristLogger, 0, 0.508753,
                        EncoderDrive.DIRECT);

            // AngularPositionServo wristAngleServo = new OnboardAngularPositionServo(
            //             wristLogger,
            //             wristMech,
            //             encoder,
            //             () -> wristProfile,
            //             wristPIDController);

            AngularPositionServo wristAngleServo = null;
            
            wristServo = new OutboardGravityServo(wristAngleServo, 5, 0);
            break;
        default:
            starboardServo = null;
            portServo = null;
            wristServo = null;



	}
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveUp(){
    
  }
}
