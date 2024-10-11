package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.handler;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.shooter;

public class intakeCommand extends Command {
    
    private intake intake;
    private handler handler;
    private shooter shooter;
    private int state;
    private boolean  finished;
    private double rollerVel, indexSpeed, shooterPos;


    public intakeCommand(intake intake, shooter shooter, handler handler) {
        this.intake = intake;
        this.shooter = shooter;
        this.handler = handler;

        addRequirements(intake, shooter);
    }
    @Override
    public void initialize(){

        rollerVel = 0.0;
        indexSpeed = 0.0;
        shooterPos = 0.46;
        intake.stopRoller();
        state = 0;
        finished = false;

    }

    @Override
    public void execute() {
        
        if(state == 0){

            shooterPos = 0.51;
            rollerVel =  100.0;
            indexSpeed = 0.9;

            if(shooter.getNoteStatus()){

                state ++;

            }

        }

        else if(state == 1){

            shooterPos = 0.46;
            indexSpeed = 0.0;
            state = 0;
            handler.setCanIntake(false);
            handler.setCanShoot(true);
            finished = true;

        }

        shooter.setShooterPosition(shooterPos);
        shooter.setIndexer(indexSpeed);
        intake.setIntakeVelocity(rollerVel);

        
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
