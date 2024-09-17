package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.drivetrain;



public class drivetrainTankCommand extends Command {

    private Supplier<Double> rTrigger, lTrigger;
    private Supplier<Boolean> isGoing;

    private drivetrain m_Drivetrain;

    private SlewRateLimiter translationLimit = new SlewRateLimiter(3.0);

    public drivetrainTankCommand(drivetrain m_Drivetrain, Supplier<Boolean> isGoing, Supplier<Double> rTrigger, Supplier<Double> lTrigger){

        this.m_Drivetrain = m_Drivetrain;

        this.isGoing = isGoing;
        this.rTrigger = rTrigger;
        this.lTrigger = lTrigger;

        addRequirements(m_Drivetrain);

    }

    @Override 
    public void execute(){
        double translation = rTrigger.get() - lTrigger.get();
        double translationVal = translationLimit.calculate(MathUtil.applyDeadband(translation, 
                                                                                  Constants.Swerve.stickDeadband));

        m_Drivetrain.drive(new Translation2d(translationVal, 0.0), 0, false, true);

    }

    @Override
    public boolean isFinished(){

        if(isGoing.get()){

            return false;
        
        }
        
        return true;

    }
    
}
