package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.handler;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.shooter;

public class shooterCommandAutoInit extends Command {

    private shooter shooter;
    private intake intake;
    private int state;
    private boolean finished;
    private double shooterVel, indexSpeed, shooterPos, startTime, intakeVel;

    public shooterCommandAutoInit(shooter shooter, intake intake){

        this.shooter = shooter;
        this.intake = intake;

        addRequirements(shooter, intake);
    }

    @Override
    public void initialize(){

        shooterVel = 20.0;
        intakeVel = 100;
        shooterPos = 0.46;
        indexSpeed = 0.0;
        startTime = 0.0;
        state = 0;
        finished = false;

    }

    @Override
    public void execute(){

        if(state == 0){

            shooterVel = 100;
            intakeVel = 100;
            shooterPos = 0.57;

            if((Math.abs(shooter.getShooterVelocity()[0])  >= 40 || Math.abs(shooter.getShooterVelocity()[1])  >= 40)){
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

            shooterVel = 20.0;
            indexSpeed = 0.0;
            shooterPos = 0.51;
            state = 0;
            finished = true;

        }

        shooter.setShooterVelocity(shooterVel);
        shooter.setIndexer(indexSpeed);
        shooter.setShooterPosition(shooterPos);
        intake.roll(intakeVel);

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
