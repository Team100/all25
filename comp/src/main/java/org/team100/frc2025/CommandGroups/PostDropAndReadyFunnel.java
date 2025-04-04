package org.team100.frc2025.CommandGroups;

import java.util.function.Supplier;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class PostDropAndReadyFunnel extends Command {
    private final Wrist2 m_wrist;
    private final Elevator m_elevator;
    private final double m_elevatorGoal;

    private double count = 0;
    private boolean finishedDrop = false;
    private boolean completelyFinished = false;
    private boolean finishedReadyFunnel = false;
    private boolean indicateReadyToLeave = false;

    private double initialElevatorPosition = 0;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    Supplier<Boolean> m_endCondition;
    
    public PostDropAndReadyFunnel(Wrist2 wrist, Elevator elevator, double elevatorValue, Supplier<Boolean> endCondition) {
        m_wrist = wrist;
        m_elevator = elevator;
        m_elevatorGoal = elevatorValue;
        m_endCondition = endCondition;
        addRequirements(m_wrist, m_elevator);
    }


    @Override
    public void initialize() {
        count = 0;
        finishedDrop = false;
        completelyFinished = false;
        finishedReadyFunnel = false;
        indicateReadyToLeave = false;
        // resetting forces the setpoint velocity to zero, which is not always what we
        // want
        // m_wrist.resetWristProfile();
        // m_elevator.resetElevatorProfile();
        

        initialElevatorPosition = m_elevator.getPosition();
    }

    @Override
    public void execute() {

        if(!finishedDrop){
            // System.out.println("FINISHED DROPPP " + finishedDrop);
            m_elevator.setPosition(m_elevatorGoal);
    
            if (Math.abs(m_elevator.getPosition() - initialElevatorPosition) > 5) {
                m_wrist.setAngleValue(0.4);
            } else {
                m_wrist.setAngleValue(1.25);
            }
    
            double error = Math.abs(m_elevator.getPosition() - m_elevatorGoal);
    
            if (error < 0.5) {
                count++;
            } else {
                count = 0;
            }
    
            if (count >= 5) {
                finishedDrop = true;
            }

            if(m_elevator.getPosition() <= 30 && m_wrist.getAngle() <= 0.9){
                indicateReadyToLeave = true;
            }
        }
       

        if(finishedDrop){

            if(m_elevator.getPosition() <= 30 && m_wrist.getAngle() <= 0.9){
                indicateReadyToLeave = true;
            }

            m_elevator.setPosition(0.1);
            m_wrist.setAngleValue(0.1);

            if( Math.abs(m_wrist.getAngle() - 0.1) < 0.05){
                if(Math.abs(m_elevator.getPosition() - 0.1 ) < 0.5){
                    finishedReadyFunnel = true;
                }
            }

            if(finishedReadyFunnel){
                m_wrist.setWristDutyCycle(-0.15);

            }

            
        }

        

    }

    public boolean indicateReadyToLeave(){
        return indicateReadyToLeave;
    }

    @Override
    public void end(boolean interrupted) {
        count = 0;
        finishedDrop = false;
        completelyFinished = false;
        finishedReadyFunnel = false;
        indicateReadyToLeave = false;

        // System.out.println("I ALREADY FINISHED");
    }

    @Override
    public boolean isFinished() {
        // if (Experiments.instance.enabled(Experiment.UseProfileDone))
        //     return completelyFinished && m_wrist.profileDone() && m_elevator.profileDone();
        // return completelyFinished;
        // System.out.println("FINISHE????  " + m_endCondition.get());
        return m_endCondition.get();
    }
}
