package frc.robot.commands;


import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.handler;

public class climberCommandDefault extends Command{


    private climber climberAna;
    private handler handler;
    private Supplier<Boolean> xButton, yButton;
    private Supplier<Double> lYAxis, rYAxis;

    public climberCommandDefault(climber climberAna, handler handler, Supplier<Boolean> xButton, Supplier<Boolean> yButton, Supplier<Double> lYAxis, Supplier<Double> rYAxis){
        this.climberAna = climberAna;
        this.handler = handler;
        this.xButton = xButton;
        this.yButton = yButton;
        this.lYAxis = lYAxis;
        this.rYAxis = rYAxis;

        addRequirements(climberAna);

    }

    @Override
    public void execute(){

        if(xButton.get()){

            climberAna.setZeroPosition();

        }

        if(yButton.get()){
        
            if(Math.abs(lYAxis.get()) > 0.2){

                climberAna.setLeftMotor(lYAxis.get() * 0.1);

            }
            else {

                climberAna.setLeftMotor(0.0);

            }

            if(Math.abs(rYAxis.get()) > 0.2){

                climberAna.setRightMotor(rYAxis.get() * 0.1);

            }
            else {

                climberAna.setRightMotor(0.0);

            }


        }

        else {

            climberAna.stopClimber();

        }

    }


}