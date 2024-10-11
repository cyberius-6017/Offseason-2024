package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climber extends SubsystemBase{

    private TalonFX motorRight;
    private TalonFX motorLeft;
    
    public climber(int climberRightID, int climberLeftID){

        motorRight = new TalonFX(climberRightID);
        motorLeft = new TalonFX(climberLeftID);

        var climberL0Configs = new Slot0Configs();
        climberL0Configs.kP = Constants.Climber.climberKP;
        climberL0Configs.kI = Constants.Climber.climberKI;
        climberL0Configs.kD = Constants.Climber.climberKD;
        motorLeft.getConfigurator().apply(climberL0Configs); 
        motorLeft.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive));
        motorLeft.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Climber.climberCurrentLImit));
        motorLeft.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(Constants.Climber.closedLoopRamp));
        motorLeft.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(Constants.Climber.climberRatio));

        var climberR0Configs = new Slot0Configs();
        climberR0Configs.kP = Constants.Climber.climberKP;
        climberR0Configs.kI = Constants.Climber.climberKI;
        climberR0Configs.kD = Constants.Climber.climberKD;
        motorRight.getConfigurator().apply(climberR0Configs); 
        motorRight.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive));
        motorRight.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Climber.climberCurrentLImit));
        motorRight.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(Constants.Climber.closedLoopRamp));
        motorLeft.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(Constants.Climber.climberRatio));
    }


    public void motorRightVelocity(double velocity){

        motorRight.setControl(new VelocityDutyCycle(velocity));


    }

    public void motorLeftVelocity(double velocity){
    
        motorLeft.setControl(new VelocityDutyCycle(velocity));
    
    }

    public void motorRightDown(double speed){

        if (speed>0){ 
                speed*=-1;}
            motorRight.set(speed);
                
        }
    
    public void motorRightUp(double speed){
        if (speed<0){ 
            speed*=-1;}
        motorRight.set(speed);
    }

    public void motorLeftDown(double speed){
        if (speed>0){ 
    
            speed*=-1;
        }
    
        motorLeft.set(speed);
    }
   
    public void motorLeftUp(double speed){

        if (speed<0){ 
        
            speed*=-1;
        
        }
        motorLeft.set(speed);

    }
        

    public void setClimberAngle(double angle){

        motorRight.setControl(new PositionDutyCycle(angle));
        motorLeft.setControl(new PositionDutyCycle(angle));

    }

    public void motorLeftAngle(double angle){
    
        motorLeft.setControl(new PositionDutyCycle(angle));
    
    }

    public void motorRightAngle(double angle){
    
        motorRight.setControl(new PositionDutyCycle(angle));
    
    }

    public double[] getClimberPosition(){

        double[] positions = {motorLeft.getPosition().getValueAsDouble(),
                              motorRight.getPosition().getValueAsDouble()};

        return positions;

    }

    @Override

    public void periodic(){

        SmartDashboard.putNumberArray("Climber Pos: ", getClimberPosition());

    }
    
    
}