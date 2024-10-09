package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.handler;
import frc.robot.subsystems.shooter;

public class shooterCommand extends Command {

    private shooter shooter;
    private handler handler;
    private Supplier<Boolean> isGoing;

    public shooterCommand(shooter shooter, handler handler, Supplier<Boolean> isGoing){

        this.shooter = shooter;
        this.handler = handler;
        this.isGoing = isGoing;

        addRequirements(shooter);
    }

    @Override
    public void execute(){

        double xPos = handler.getRobotPose().getX();
        double shooterPos = 0.0;

        shooterPos =handler.mapd(xPos, 1.30, 2.40, 0.58, 0.46);

        shooter.setShooterPosition(shooterPos);

    }
    @Override
    public boolean isFinished(){

        if(isGoing.get()){
        
            return false;

            
        }
        LimelightHelpers.setLEDMode_ForceOff(Constants.Sensors.limef);
        return true;
    }
    
}
