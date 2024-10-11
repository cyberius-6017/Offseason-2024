package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;

public class intakeCommandDefault extends Command{

    private intake intake;

    public intakeCommandDefault(intake intake){

        this.intake = intake;

        addRequirements(intake);
    }
    
    @Override
    public void execute(){

        intake.stopRoller();

    }
}
