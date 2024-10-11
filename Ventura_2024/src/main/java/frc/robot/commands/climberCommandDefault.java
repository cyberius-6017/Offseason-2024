package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.handler;

public class climberCommandDefault extends Command{


    private climber climberAna;
    private handler handler;

    public climberCommandDefault(climber climberAna, handler handler)
    {
        this.climberAna = climberAna;
        this.handler = handler;

        addRequirements(climberAna);

    }
    @Override
    public void execute(){
        
        

    }


}