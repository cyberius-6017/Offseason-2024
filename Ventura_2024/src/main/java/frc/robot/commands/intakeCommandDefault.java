package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;

public class intakeCommandDefault extends Command{

    private intake intake;
    private Supplier<Boolean> bButton;

    public intakeCommandDefault(intake intake, Supplier<Boolean> bButton){

        this.intake = intake;
        this.bButton = bButton;

        addRequirements(intake);
    }
    
    @Override
    public void execute(){

        if(bButton.get()){

            intake.setIntakeVelocity(-120);

        }

        else{

            intake.stopRoller();

        }

    }
}
