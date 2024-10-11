package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climber extends SubsystemBase{
    private TalonFX MotorRight;
    private TalonFX MotorLeft;
    public climber(int motor_ID){}


public void MotorRightVelocity(double velocity){
       MotorRight.setControl(new VelocityDutyCycle(velocity));

}
public void MotorRightAngle(double angle){

}

public void MotorRightDown(double speed){
   if (speed>0){ 
        speed*=-1;}
    MotorRight.set(speed);
        
}

public void MotorRightUp(double speed){
    if (speed<0){ 
        speed*=-1;}
    MotorRight.set(speed);
}
 
public void MotorLeftVelocity(double velocity){
MotorLeft.setControl(new VelocityDutyCycle(velocity));
}
public void MotorLeftAngle(double angle){
    MotorLeft.setControl(new PositionDutyCycle(angle));
}
public void MotorLeftDown(double speed){
    if (speed>0){ 
        speed*=-1;}
    MotorLeft.set(speed);
}
public void MotorLeftUp(double speed){
    if (speed<0){ 
        speed*=-1;}
    MotorLeft.set(speed);

}



}