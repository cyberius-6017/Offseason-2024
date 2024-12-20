package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.handler;
import frc.robot.subsystems.drivetrain.drivetrain;

public class drivetrainCommandAlignShuttle extends Command {
    
    private drivetrain driveTrain;
    private handler handler;

    private Supplier<Double> stickX, stickY;
    private Supplier<Boolean> isGoing;
    private Translation2d deltaPos, robotPos;
    private Rotation2d currentRot;

    private SlewRateLimiter translationLimit = new SlewRateLimiter(2.0);
    private SlewRateLimiter strafeLimit = new SlewRateLimiter(2.0);

    public drivetrainCommandAlignShuttle(drivetrain drivetrain, handler handler, Supplier<Double> stickX, Supplier<Double> stickY, Supplier<Boolean> isGoing){
        this.driveTrain = drivetrain;
        this.stickX = stickX;
        this.stickY = stickY;
        this.handler = handler;
        this.isGoing = isGoing;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Optional<Alliance> ally = DriverStation.getAlliance();

        robotPos = driveTrain.getPose().getTranslation();
        deltaPos = robotPos.minus(Constants.Field.shuttleBlue);
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                deltaPos = robotPos.minus(Constants.Field.shuttleRed);
            }
            if (ally.get() == Alliance.Blue) {
                deltaPos = robotPos.minus(Constants.Field.shuttleBlue);
            }
        }
        currentRot = driveTrain.getPose().getRotation();
        
        double setPoint = deltaPos.getAngle().getDegrees();
        double error = setPoint - currentRot.getDegrees();


        if(Math.abs(error) < 1.0) {

            error = 0.0;
            LimelightHelpers.setLEDMode_ForceBlink(Constants.Sensors.limef);

        }

        if(error < -300){

            System.out.println("Aqui -300");
            error += 360.0;

        }
        else if(error > 300){

            System.out.println("Aqui 300");
            error -= 360.0;

        }

        // System.out.print("Setpoint: " + setPoint + " ");
        // System.out.println("Error: " + error);

         double translationVal = translationLimit.calculate(MathUtil.applyDeadband(stickX.get(), 
                                                                                  Constants.Swerve.stickDeadband));
        double strafeVal = strafeLimit.calculate(MathUtil.applyDeadband(stickY.get(), 
                                                                        Constants.Swerve.stickDeadband));

        driveTrain.alignRobotSpeaker(translationVal, strafeVal, error * Constants.Swerve.alignSpkKP);
    }

    @Override
    public void end(boolean interrupted){
    
        LimelightHelpers.setLEDMode_ForceOff(Constants.Sensors.limef);
        
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
