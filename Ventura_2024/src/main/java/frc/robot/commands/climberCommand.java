package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber;

public class climberCommand extends Command{


    private climber climberAna;

    public climberCommand(climber climberAna){
        this.climberAna=climberAna;

    addRequirements(climberAna);


    }
    @Override
    public void execute(){
        if (climberAna.getRightClimberPosition>0)
            climberAna.getRightClimberPosition*-1
        E
        
        climberAna.MotorRightAngle(90);
        climberAna.MotorLeftAngle(90);

    }


}