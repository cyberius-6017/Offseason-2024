package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.drivetrain;

public class drivetrainCommandAlignSpeaker extends Command {
    
    private drivetrain driveTrain;
    private Supplier<Double> stickX, stickY;
    private Supplier<Boolean> isGoing;
    private double angle;
    private Translation2d delta, roboPos;

    public drivetrainCommandAlignSpeaker(drivetrain drivetrain, Supplier<Double> stickX, Supplier<Double> stickY, Supplier<Boolean> isGoing){
        this.driveTrain = drivetrain;
        this.stickX = stickX;
        this.stickY = stickY;
        this.isGoing = isGoing;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        roboPos = driveTrain.getPose().getTranslation();
        delta = roboPos.minus(Constants.Field.speakBlue);
        angle = delta.getAngle().getDegrees();

        if(angle > 0){

            angle *= Constants.Swerve.alignSpkKP;

        }

        else {

            angle *= -Constants.Swerve.alignSpkKP;

        }

        if(Math.abs(angle) < 0.1) {

            angle = 0.0;

        }

        driveTrain.alignRobotSpeaker(stickX.get(), stickY.get(), angle);
    }

    @Override
    public boolean isFinished(){

        if(isGoing.get()){
        
            return false;

            
        }
        return true;
    }    

}
