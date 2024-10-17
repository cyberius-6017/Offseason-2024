package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
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
        climberL0Configs.kP = Constants.Climber.climberKPUp;
        climberL0Configs.kI = Constants.Climber.climberKIUp;
        climberL0Configs.kD = Constants.Climber.climberKDUp;
        var climberL1Configs = new Slot1Configs();
        climberL1Configs.kP = Constants.Climber.climberKPDo;
        climberL1Configs.kI = Constants.Climber.climberKIDo;
        climberL1Configs.kD = Constants.Climber.climberKDDo;
        motorLeft.getConfigurator().apply(climberL0Configs); 
        motorLeft.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive));
        motorLeft.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Climber.climberCurrentLImit));
        motorLeft.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(Constants.Climber.closedLoopRamp));
        motorLeft.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(Constants.Climber.climberRatio));

        var climberR0Configs = new Slot0Configs();
        climberR0Configs.kP = Constants.Climber.climberKPUp;
        climberR0Configs.kI = Constants.Climber.climberKIUp;
        climberR0Configs.kD = Constants.Climber.climberKDUp;
        var climberR1Configs = new Slot1Configs();
        climberR1Configs.kP = Constants.Climber.climberKPDo;
        climberR1Configs.kI = Constants.Climber.climberKIDo;
        climberR1Configs.kD = Constants.Climber.climberKDDo;
        motorLeft.getConfigurator().apply(climberL0Configs); 
        motorRight.getConfigurator().apply(climberR0Configs); 
        motorRight.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.CounterClockwise_Positive));
        motorRight.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Climber.climberCurrentLImit));
        motorRight.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(Constants.Climber.closedLoopRamp));
        motorRight.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(Constants.Climber.climberRatio));
    
        setZeroPosition();

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

    public void setRightMotor(double speed){

        motorRight.set(-speed);

    }

    public void setLeftMotor(double speed){

        motorLeft.set(-speed);

    }
   
    public void motorLeftUp(double speed){

        if (speed<0){ 
        
            speed*=-1;
        
        }
        motorLeft.set(speed);

    }
        

    public void setClimberAngle(double angle, int slot, double feedforward){

        motorRight.setControl(new PositionDutyCycle(angle).withSlot(slot).withFeedForward(feedforward));
        motorLeft.setControl(new PositionDutyCycle(angle).withSlot(slot).withFeedForward(feedforward));

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

    public void setZeroPosition(){

        motorLeft.setPosition(0.0);
        motorRight.setPosition(0.0);
        System.out.println("Climbers zeroed correctly");

    }

    public double[] getClimberDC(){

        double[] dcs = {motorLeft.getDutyCycle().getValueAsDouble(),
                        motorRight.getDutyCycle().getValueAsDouble()};

        return dcs;

    }
    public void stopClimber(){

        motorLeft.stopMotor();
        motorRight.stopMotor();

    }

    @Override

    public void periodic(){

        SmartDashboard.putNumberArray("Climber Pos: ", getClimberPosition());

    }
    
    
}