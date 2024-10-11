package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class arduinoComm extends SubsystemBase {

    private SerialPort arduino;


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
}
