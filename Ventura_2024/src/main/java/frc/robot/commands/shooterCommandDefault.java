package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.handler;
import frc.robot.subsystems.shooter;

public class shooterCommandDefault  extends Command{

    private shooter shooter;
    private handler handler;
    private Supplier<Boolean> rBumper, lBumper;

    public shooterCommandDefault(shooter shooter, handler handler, Supplier<Boolean> rBumper, Supplier<Boolean> lBumper){

        this.shooter = shooter;
        this.handler = handler;
        this.rBumper = rBumper;
        this.lBumper = lBumper;

        addRequirements(shooter);

    }

    @Override
    public void execute(){

        
        shooter.setShooterPosition(0.46);
        shooter.setIndexer(0.0);
        shooter.setShooterVelocity(20);

        if(rBumper.get()){

            handler.setCanIntake(false);
            handler.setCanShoot(true);

        }

        if(lBumper.get()){

            handler.setCanIntake(true);
            handler.setCanShoot(false);

        }

    }
    
    
}
