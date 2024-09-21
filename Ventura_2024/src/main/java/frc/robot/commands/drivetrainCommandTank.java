package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.drivetrain;

public class drivetrainCommandTank extends Command {

    private drivetrain drivetrain;

    private Supplier<Double> rTrigger, lTrigger;
    private Supplier<Boolean> isGoing;

    public drivetrainCommandTank(drivetrain drivetrain, Supplier<Double> rTrigger, Supplier<Double> lTrigger, Supplier<Boolean> isGoing){

        this.drivetrain = drivetrain;
        this.rTrigger = rTrigger;
        this.lTrigger = lTrigger;
        this.isGoing = isGoing;

        addRequirements(drivetrain);

    }

    @Override
    public void execute(){

        drivetrain.drive(new Translation2d(rTrigger.get()- lTrigger.get() , 0.0), 0.0, false, true);

    }

    @Override
    public boolean isFinished(){

        if(isGoing.get()){

            return false;
        
        }
        
        return true;

    }
    
}
