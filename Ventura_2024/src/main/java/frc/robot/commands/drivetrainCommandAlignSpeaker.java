package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drivetrain.drivetrain;

public class drivetrainCommandAlignSpeaker extends Command {
    
    private drivetrain driveTrain;
    private Supplier<Double> stickX, stickY;
    private Supplier<Boolean> isGoing;
    private Translation2d deltaPos, robotPos;
    private Rotation2d currentRot;

    public drivetrainCommandAlignSpeaker(drivetrain drivetrain, Supplier<Double> stickX, Supplier<Double> stickY, Supplier<Boolean> isGoing){
        this.driveTrain = drivetrain;
        this.stickX = stickX;
        this.stickY = stickY;
        this.isGoing = isGoing;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        
        robotPos = driveTrain.getPose().getTranslation();
        deltaPos = robotPos.minus(Constants.Field.speakBlue);
        currentRot = driveTrain.getPose().getRotation();
        
        double setPoint = deltaPos.getAngle().getDegrees();
        double error = setPoint - currentRot.getDegrees();


        if(Math.abs(error) < 0.3) {

            error = 0.0;
            LimelightHelpers.setLEDMode_ForceBlink(Constants.Sensors.limef);

        }
        // System.out.print("Setpoint: " + setPoint + " ");
        // System.out.println("Error: " + error);

        driveTrain.alignRobotSpeaker(stickX.get(), stickY.get(), error * Constants.Swerve.alignSpkKP);
    }

    @Override
    public boolean isFinished(){

        if(isGoing.get()){
        
            return false;

            
        }
        LimelightHelpers.setLEDMode_ForceOff(Constants.Sensors.limef);
        return true;
    }    

}
