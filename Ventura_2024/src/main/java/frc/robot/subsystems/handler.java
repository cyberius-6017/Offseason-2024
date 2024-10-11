package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class handler extends SubsystemBase{

    public static boolean canIntake;
    public static boolean canShoot;
    public static boolean canAlignSpeaker;
    public static boolean canAlignDrive;
    public static boolean canClimb;
    public static Pose2d robotPos;
    private SerialPort arduino;

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

  public void ledMode(String mode) {
    if (mode.equals("idle")) {
      arduino.write(new byte[] {0x00}, 1);

    } else if (mode.equals("noNote")) {
      arduino.write(new byte[] {0x01}, 1);
    } else if (mode.equals("noteIn")) {
      arduino.write(new byte[] {0x02}, 1);
    } else if (mode.equals("shootReady")) {
      arduino.write(new byte[] {0x03}, 1);
    } else if (mode.equals("climb")) {
      arduino.write(new byte[] {0x04}, 1);
    }
  }

 public double mapd(double x, double in_min, double in_max, double out_min, double out_max) {

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  
  }

  public void setupArduino() {
    try {
      arduino = new SerialPort(9600, SerialPort.Port.kUSB);
      System.out.println("Connected on kUSB!");
    } catch (Exception e) {
      System.out.println("Failed to connect on kUSB, trying kUSB 1");

      try {
        arduino = new SerialPort(9600, SerialPort.Port.kUSB1);
        System.out.println("Connected on kUSB1!");
      } catch (Exception e1) {
        System.out.println("Failed to connect on kUSB1, trying kUSB 2");

        try {
          arduino = new SerialPort(9600, SerialPort.Port.kUSB2);
          System.out.println("Connected on kUSB2!");
        } catch (Exception e2) {
          System.out.println("Failed to connect on kUSB2, all connection attempts failed!");
        }
      }
    }
  }

  @Override
  public void periodic() {
      SmartDashboard.putBoolean("Can Intake: ", canIntake);
      SmartDashboard.putBoolean("Can Shoot: ", canShoot);
      SmartDashboard.putBoolean("Can Climb:", canClimb);
  }

  

}

