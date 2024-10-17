package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drivetrain.drivetrain;

public class drivetrainCommandAlignClimb extends Command{


    private drivetrain drivetrain;
    private Supplier<Boolean> isGoing;
    private Pose2d robotPose;
    private Rotation2d desPos, currentRot;
    private Supplier<Double> stickX, stickY;
    
    private SlewRateLimiter translationLimit = new SlewRateLimiter(2.0);
    private SlewRateLimiter strafeLimit = new SlewRateLimiter(2.0);

    public drivetrainCommandAlignClimb(drivetrain drivetrain, Supplier<Double> stickX, Supplier<Double> stickY, Supplier<Boolean> isGoing){

        this.drivetrain = drivetrain;
        this.stickX = stickX;
        this.stickY = stickY;
        this.isGoing = isGoing;

    }
    @Override
    public void execute(){

        robotPose = drivetrain.getPose();
        currentRot = drivetrain.getPose().getRotation();

        Optional<Alliance> ally = DriverStation.getAlliance();
        
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {

                if(robotPose.getX() < 11.7){

                    desPos = Rotation2d.fromDegrees(180);

                }

                else {

                    if(robotPose.getY() > 4.6){

                        desPos = Rotation2d.fromDegrees(45);

                    }
                    else {

                        desPos = Rotation2d.fromDegrees(325);

                    }

                }
                        
            }
            else if (ally.get() == Alliance.Blue) {

                if(robotPose.getX() > 6.16){

                    desPos = Rotation2d.fromDegrees(180);

                }
                else {

                    if(robotPose.getY() < 4.6){

                        desPos = Rotation2d.fromDegrees(45);

                    }
                    else {

                        desPos = Rotation2d.fromDegrees(325);

                    }

                }
                
            }

            double error = desPos.getDegrees() - currentRot.getDegrees();


            if(Math.abs(error) < 0.3) {

                error = 0.0;
                LimelightHelpers.setLEDMode_ForceBlink(Constants.Sensors.limef);

            }

            double translationVal = translationLimit.calculate(MathUtil.applyDeadband(stickX.get(), 
                                                                                      Constants.Swerve.stickDeadband));
            double strafeVal = strafeLimit.calculate(MathUtil.applyDeadband(stickY.get(), 
                                                                            Constants.Swerve.stickDeadband));

            drivetrain.alignRobotSpeaker(translationVal, strafeVal, error * Constants.Swerve.alignSpkKP);

        }

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
