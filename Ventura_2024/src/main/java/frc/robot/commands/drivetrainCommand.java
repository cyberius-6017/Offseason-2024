package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.drivetrain;

public class drivetrainCommand extends Command{

    private drivetrain driveTrain;
    private Supplier<Double> translation, strafe, rotation;
    private Supplier<Boolean> changeDrive, resetPigeon;
    private boolean isRobotCentric = false;

    
    private SlewRateLimiter translationLimit = new SlewRateLimiter(3.0);
    private SlewRateLimiter strafeLimit = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimit = new SlewRateLimiter(3.0);

    

    public drivetrainCommand(drivetrain m_DriveTrain, Supplier<Double> translation, Supplier<Double> strafe, Supplier<Double> rotation, Supplier<Boolean> changeDrive, Supplier<Boolean> resetPigeon){

        this.driveTrain = m_DriveTrain;
        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
        this.changeDrive = changeDrive;
        this.resetPigeon = resetPigeon;
        isRobotCentric = true;

        addRequirements(m_DriveTrain);

    }
    
    @Override
    public void execute() {

        if(changeDrive.get()){
            
            isRobotCentric = !isRobotCentric;

        }    

        if(resetPigeon.get()){

            driveTrain.zeroPigeon();

        }    

        double translationVal = translationLimit.calculate(MathUtil.applyDeadband(translation.get(), 
                                                                                  Constants.Swerve.stickDeadband));
        double strafeVal = strafeLimit.calculate(MathUtil.applyDeadband(strafe.get(), 
                                                                        Constants.Swerve.stickDeadband));
        double rotationVal = rotationLimit.calculate(MathUtil.applyDeadband(rotation.get(), 
                                                                            Constants.Swerve.stickDeadband));
        SmartDashboard.putString("Drive Mode:",  (isRobotCentric ? "Driving Field Oriented"
                                                                     : "Driving Robot Oriented"));
        driveTrain.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), rotationVal * Constants.Swerve.maxAngularVelocity, isRobotCentric, true);
    }
}