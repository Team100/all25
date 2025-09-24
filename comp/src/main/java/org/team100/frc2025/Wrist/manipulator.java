package org.team100.frc2025.Wrist;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase{
    private final OutboardLinearVelocityServo m_leftCoral;
    private final Kraken6Motor m_leftMotor;
    private final Kraken6Motor m_rightMotor;
    private final Kraken6Motor m_algaeMotor;
    private final LaserCan m_coralRightLaserCan;
    private final LaserCan m_coralCenterLaserCan;
    private final LaserCan m_coralBackLaserCan;
    private final LaserCan m_coralLeftLaserCan;
    
    public Manipulator(LoggerFactory log) {
        m_leftMotor = new Kraken6Motor(log, 54, MotorPhase.FORWARD, 40, 40, new PIDConstants(), Feedforward100.makeShooterFalcon6());
        m_rightMotor = new Kraken6Motor(log, 55, MotorPhase.REVERSE, 40, 40, new PIDConstants(), Feedforward100.makeShooterFalcon6());
        m_algaeMotor = new Kraken6Motor(log, 56, MotorPhase.FORWARD, 120, 120 , new PIDConstants(), Feedforward100.makeShooterFalcon6());
        m_coralRightLaserCan = new LaserCan(52);
        m_coralCenterLaserCan = new LaserCan(51);
        m_coralBackLaserCan = new LaserCan(53);
        m_coralLeftLaserCan = new LaserCan(50);
        LinearMechanism linearMechanism = new LinearMechanism(log, m_leftMotor, new Talon6Encoder(log, m_leftMotor), 16, .1, -100000000, 1000000);
        //LinearMechanism linearMechanism = new LinearMechanism(log, m_leftMotor, new Talon6Encoder(log, m_leftMotor), 16, .1, -100000000, 1000000);
        //LinearMechanism linearMechanism = new LinearMechanism(log, m_leftMotor, new Talon6Encoder(log, m_leftMotor), 16, .1, -100000000, 1000000);
        m_leftCoral = new OutboardLinearVelocityServo(log, linearMechanism);
    }

    public boolean isCoralClose(double distance) {
        return distance < 100;
    }
    public void intakeCenter(){
        if(isCoralClose(m_coralBackLaserCan.getMeasurement().distance_mm)){
         stop();}
        else{
            m_algaeMotor.setDutyCycle(-0.5);
            m_leftMotor.setDutyCycle(0.5);
            m_rightMotor.setDutyCycle(0.5);
        }  
    }
    

    public void intakeSideways(){
        if(isCoralClose(m_coralLeftLaserCan.getMeasurement().distance_mm)&&isCoralClose(m_coralRightLaserCan.getMeasurement().distance_mm)){
           stop();
        }
        else {
            m_algaeMotor.setDutyCycle(-0.5);
                if(isCoralClose(m_coralLeftLaserCan.getMeasurement().distance_mm)){
                    m_leftMotor.setDutyCycle(0.5);
                    m_rightMotor.setDutyCycle(-0.5);
                }
                else{
                    m_leftMotor.setDutyCycle(-0.5);
                    m_rightMotor.setDutyCycle(0.5);
                }
        }
    }

    public void stop(){
        m_algaeMotor.setDutyCycle(0);
        m_leftMotor.setDutyCycle(0);
        m_rightMotor.setDutyCycle(0);
    }

    public void intakeAlgae(){
        if(m_algaeMotor.getCurrent()>80){
        m_algaeMotor.setDutyCycle(0.5);
        }else{
        m_algaeMotor.setDutyCycle(1);
}

}
}
