package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arduinoComm;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.handler;

public class arduinoCommunicationCommand extends Command {

    private arduinoComm arduino;
    private handler handler;

    public arduinoCommunicationCommand(arduinoComm arduino, handler handler) {
        this.arduino = arduino;
        this.handler = handler;

        addRequirements(arduino);
    }

    @Override
    public void execute(){
        if(handler.getShootReady()) {
            arduino.ledMode("shootReady");
        } else if(handler.getCanIntake()) {
            arduino.ledMode("noNote");
        } else if(handler.getCanShoot()) {
            arduino.ledMode("noteIn");
        }
    }
}
