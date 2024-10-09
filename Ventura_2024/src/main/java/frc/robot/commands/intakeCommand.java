package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.shooter;

public class intakeCommand extends Command {
    
    private intake intake;
    private shooter shooter;
    private Supplier<Boolean> isGoing;

    public intakeCommand(intake intake, shooter shooter,  Supplier<Boolean> isGoing) {
        this.intake = intake;
        this.shooter = shooter;
        this.isGoing = isGoing;

        addRequirements(intake, shooter);
    }

    @Override
    public void execute() {
        
        intake.roll(100);
        shooter.setShooterPosition(0.51);
        shooter.setIndexer(1.0);
        shooter.setShooterVelocity(100);
    }

    @Override
    public boolean isFinished(){

        if(isGoing.get()){
        
            return false;

            
        }
        return true;
    }   
}
