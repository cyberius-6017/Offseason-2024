package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.handler;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.shooter;

public class intakeCommandAuto extends Command {
    
    private intake intake;
    private shooter shooter;
    private int state;
    private boolean  finished;
    private double rollerVel, indexSpeed, shooterPos, startTime;


    public intakeCommandAuto(intake intake, shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;

        addRequirements(intake, shooter);
    }
    @Override
    public void initialize(){

        rollerVel = 0.0;
        startTime = 0.0;
        indexSpeed = 0.0;
        shooterPos = 0.46;
        intake.stopRoller();
        state = -1;
        finished = false;

    }

    @Override
    public void execute() {
        if(state == -1){

            startTime = Timer.getFPGATimestamp();
            state++;

        }
        
        else if(state == 0){

            shooterPos = 0.51;
            rollerVel =  120.0;
            indexSpeed = 0.25;
    

            if(shooter.getNoteStatus() || Timer.getFPGATimestamp() - startTime > 5  ){

                state ++;

            }

        }

        else if(state == 1){

            shooterPos = 0.46;
            startTime = 0.0;
            indexSpeed = 0.0;
            state = -1;

            
            finished = true;

        }

        shooter.setShooterPosition(shooterPos);
        shooter.setIndexer(indexSpeed);
        intake.setIntakeVelocity(rollerVel);

        
    }

    @Override
    public void end(boolean interrupted){

        startTime = 0.0;
        state = -1;
    }

    @Override
    public boolean isFinished(){

        if(finished){
        
            return true;
            
        }
        return false;
    }   
}
