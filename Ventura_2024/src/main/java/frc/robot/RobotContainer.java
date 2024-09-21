package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drivetrainCommandDefault;
import frc.robot.commands.drivetrainCommandTank;
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

    autoChooser = AutoBuilder.buildAutoChooser("EZ AUTO");
    autoChooser.addOption("EZ AUTO", new PathPlannerAuto("EZ AUTO"));
    autoChooser.addOption("Test", new PathPlannerAuto("Test"));

    m_Drivetrain.setDefaultCommand(new drivetrainCommandDefault(m_Drivetrain, 
                                   ()-> -driverController.getLeftY(), 
                                   ()-> -driverController.getLeftX(),
                                   ()-> -driverController.getRightX(),
                                   ()-> driverController.getBButtonPressed(),
                                   ()-> driverController.getYButtonPressed()));


                        

    configureBindings();

    SmartDashboard.putData("Autonomo",autoChooser);
  }

  private void configureBindings() {

    tankTrigger.onTrue(new drivetrainCommandTank(m_Drivetrain, 
                                                 ()-> driverController.getRightTriggerAxis(), 
                                                 ()-> driverController.getLeftTriggerAxis(),
                                                 ()-> (Math.abs(driverController.getRightTriggerAxis()) > 0.2 
                                                    || Math.abs(driverController.getLeftTriggerAxis()) > 0.2)));
    
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return  autoChooser.getSelected();
  }
}
