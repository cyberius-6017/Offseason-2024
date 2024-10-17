package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class handler extends SubsystemBase{

    public static boolean canIntake;
    public static boolean canShoot;
    public static boolean canAlignSpeaker;
    public static boolean canAlignDrive;
    public static boolean canClimb;
    public static boolean shootReady;
    public static Pose2d robotPos;


    public handler(){

        canIntake = true;
        canShoot = false;
        canAlignSpeaker = false;
        canAlignDrive = false;
        canClimb = false;

    }

  public void setRobotPose(Pose2d pose){

    robotPos = pose;    

  }  

  public Pose2d getRobotPose(){
    
    return robotPos;

  }

  ///////////////////////////////////////
  public void setCanIntake(boolean isOn){
    canIntake = isOn;  
  }
  public void toggleCanIntake(){
    canIntake = !canIntake;
  }
  public boolean getCanIntake(){
    return canIntake;
  }
  ///////////////////////////////////////
  public void setCanShoot(boolean isOn){
    canShoot = isOn;  
  }
  public void toggleCanShoot(){
    canShoot = !canShoot;
  }
  public boolean getCanShoot(){
    return canShoot;
  }
   ///////////////////////////////////////
  public void setShootReady(boolean isOn){
    shootReady = isOn;  
  }
  public void toggleShootReady(){
    shootReady = !shootReady;
  }
  public boolean getShootReady(){
    return shootReady;
  }

  ///////////////////////////////////////////
  public void setCanAlignSpeaker(boolean isOn){
    canAlignSpeaker = isOn;  
  }
  public void toggleCanAlignSpeaker(){
    canAlignSpeaker= !canAlignSpeaker;
  }
  ////////////////////////////////////////////
  public void setCanAlignDrive(boolean isOn){
    canAlignDrive = isOn;  
  }
  public void toggleCanAlignDrive(){
    canAlignDrive= !canAlignDrive;
  }
  public boolean getCanAlignDrive(){
    return canAlignDrive;
  }
  ////////////////////////////////////////////
 
  public void setCanClimb(boolean isOn){
    canClimb = isOn;  
  }
  public void toggleCanClimb(){
    canClimb= !canClimb;
  }
  public boolean getCanClimb(){
    return canClimb;
  }
  ///////////////////////////////////////////

 public double mapd(double x, double in_min, double in_max, double out_min, double out_max) {

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  
  }

  public double setRegression(double inter, double alpha, double beta,  double x1, double x2) {

    return inter + alpha * x1 + beta * x2;
  
  }

  @Override
  public void periodic() {
      SmartDashboard.putBoolean("Can Intake: ", canIntake);
      SmartDashboard.putBoolean("Can Shoot: ", canShoot);
      SmartDashboard.putBoolean("Can Climb:", canClimb);
  }

  

}

