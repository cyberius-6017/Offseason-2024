package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.handler;
import frc.robot.subsystems.shooter;

public class shooterCommandAuto extends Command {

    private shooter shooter;
    private int state;
    private boolean finished;
    private double shooterVel, indexSpeed, shooterPos, startTime;

    public shooterCommandAuto(shooter shooter){

        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize(){

        shooterVel = 20.0;
        shooterPos = 0.46;
        indexSpeed = 0.0;
        startTime = 0.0;
        state = 0;
        finished = false;

    }

    @Override
    public void execute(){

        if(state == 0){

            indexSpeed = 0.0;
            shooterVel = 100;
            shooterPos = 0.53;

            if((Math.abs(shooter.getShooterVelocity()[0])  >= 50 || Math.abs(shooter.getShooterVelocity()[1])  >= 50)){
            //handler.setShootReady(true);
                //Blink
                
                    startTime = Timer.getFPGATimestamp();
                    state ++;

                

            }

        }    

        else if(state == 1){

            indexSpeed = 1.0;

            if(Timer.getFPGATimestamp() - startTime > 0.5){

                startTime = 0.0;
                state ++;

            }


        }

        else if(state == 2){

            shooterVel = 15.0;
            indexSpeed = 0.0;
            //TODO ctrl+z
            shooterPos = 0.46;
            state = 0;
            finished = true;

        }

        shooter.setShooterVelocity(shooterVel);
        shooter.setIndexer(indexSpeed);
        shooter.setShooterPosition(shooterPos);

    }

    @Override
    public void end(boolean interrupted){
    
        state = 0;
    }

    @Override
    public boolean isFinished(){

        if(finished){
        
            return true;
            
        }
        return false;
    }  
    
}
