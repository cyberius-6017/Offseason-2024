package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;

public class intakeCommandDefault extends Command{

    private intake intake;    
    private Supplier<Double> rTrigger, lTrigger;

    public intakeCommandDefault(intake intake,Supplier<Double> rTrigger, Supplier<Double> lTrigger){

        this.intake = intake;
        this.rTrigger = rTrigger;
        this.lTrigger = lTrigger;

        addRequirements(intake);
    }
    
    @Override
    public void execute(){

        intake.stopRoller();

    }
}
