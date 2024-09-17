package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drivetrainCommand;
import frc.robot.commands.drivetrainTankCommand;
import frc.robot.subsystems.drivetrain.drivetrain;

public class RobotContainer {

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final XboxController driverController = new XboxController(Constants.OperatorConstants.driverDriveTrainPort);

  private final drivetrain m_Drivetrain = new drivetrain();

  private Trigger tankTrigger = new Trigger((()-> Math.abs(driverController.getRightTriggerAxis()) > 0.2))
                            .or(new Trigger((()-> Math.abs(driverController.getLeftTriggerAxis()) > 0.2)));

  public void registerCommands(){

  }
  
  public RobotContainer() {
    
    registerCommands();

    m_Drivetrain.setDefaultCommand(new drivetrainCommand(m_Drivetrain, 
                                   ()-> -driverController.getLeftX(), 
                                   ()-> driverController.getLeftY(),
                                   ()-> -driverController.getRightX(),
                                   ()-> driverController.getBButtonPressed(),
                                   ()-> driverController.getYButtonPressed()));


                        

    configureBindings();

    SmartDashboard.putData("Autonomo",autoChooser);
  }

  private void configureBindings() {

    tankTrigger.onTrue(new drivetrainTankCommand(m_Drivetrain, (()-> (Math.abs(driverController.getRightTriggerAxis()) > 0.2 || Math.abs(driverController.getLeftTriggerAxis()) > 0.2)), 
                                                                ()-> driverController.getRightTriggerAxis(), 
                                                                ()-> driverController.getLeftTriggerAxis()));
    
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
