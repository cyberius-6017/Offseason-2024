package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.handler;
import frc.robot.subsystems.shooter;

public class climberCommand extends Command{

    private climber climber;
    private handler handler;
    private shooter shooter;
    private boolean finished;
    private int state;
    private double startTime;

    public climberCommand(climber climber, handler handler, shooter shooter){

        
        this.climber = climber;
        this.handler = handler;
        this.shooter = shooter;

        addRequirements(climber);

    }
    @Override
    public void initialize(){

        startTime = 0.0;
        state = 0;
        finished = false;

    }

    @Override
    public void execute(){
        if(state == 0){

            startTime = Timer.getFPGATimestamp();
            state ++;
           
        }

        else if(state == 1){

            if(!handler.getCanClimb()){

                climber.setClimberAngle(2.90);
                if(Timer.getFPGATimestamp() - startTime > 0.5 && (Math.abs(climber.getClimberDC()[0]) < 0.01 || Math.abs(climber.getClimberDC()[1]) < 0.01)){

                    handler.setCanClimb(true);
                    state ++;

                }

            }

            else if(handler.getCanClimb()){

                climber.setClimberAngle(0.05);
                shooter.setShooterPosition(0.58);
                System.out.println(Math.abs(climber.getClimberDC()[1]));
                if(Timer.getFPGATimestamp() - startTime > 0.5 && (Math.abs(climber.getClimberDC()[0]) < 0.042 || Math.abs(climber.getClimberDC()[1]) < 0.042)){
                    shooter.setShooterPosition(0.58);
                    // handler.setCanClimb(false);
                    // finished = true;
                    // state ++;

                }

            }

        }

        else if(state == 2) {

            startTime = 0.0;
            state = 0;
            finished = true;

        }

    }

    @Override
    public void end(boolean interrupted){
        startTime = 0.0;
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
