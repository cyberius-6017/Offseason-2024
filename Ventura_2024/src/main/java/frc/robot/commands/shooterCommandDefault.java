package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.handler;
import frc.robot.subsystems.shooter;

public class shooterCommandDefault  extends Command{

    private shooter shooter;
    //private handler handler;

    public shooterCommandDefault(shooter shooter, handler handler){

        this.shooter = shooter;
        //this.handler = handler;

        addRequirements(shooter);

    }

    @Override
    public void execute(){

        //System.out.println(handler.getRobotPose());
        shooter.setShooterPosition(0.46);
        shooter.setIndexer(0.0);
        shooter.setShooterVelocity(100);

    }
    
    
}
