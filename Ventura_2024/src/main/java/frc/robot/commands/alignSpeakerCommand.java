package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.drivetrain;

public class alignSpeakerCommand extends Command {
    
    private drivetrain driveTrain;
    private Supplier<Double> stickX, stickY;
    private Supplier<Boolean> isGoing;
    private double deltaA;
    private Rotation2d angle;
    private Translation2d delta, roboPos;

    public alignSpeakerCommand(drivetrain drivetrain, Supplier<Double> stickX, Supplier<Double> stickY, Supplier<Boolean> isGoing){
        this.driveTrain = drivetrain;
        this.stickX = stickX;
        this.stickY = stickY;
        this.isGoing = isGoing;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        roboPos = driveTrain.getSwervePose().getTranslation();
        delta = roboPos.minus(Constants.Field.speakBlue);
        angle = delta.getAngle();
        System.out.println("Angle: " + angle.getDegrees());

        driveTrain.alignRobotSpeaker(stickX.get(), stickY.get(), angle.getDegrees());
    }

    @Override
    public boolean isFinished(){

        if(isGoing.get()){
        
            return false;

            
        }
        return true;
    }    

}
