package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drivetrainCommandAlignShuttle;
import frc.robot.commands.drivetrainCommandAlignSpeaker;
import frc.robot.commands.drivetrainCommandDefault;
import frc.robot.commands.drivetrainCommandTank;
import frc.robot.commands.intakeCommand;
import frc.robot.commands.intakeCommandDefault;
import frc.robot.subsystems.intake;
// import frc.robot.commands.drivetrainCommand;
// import frc.robot.commands.drivetrainTankCommand;
import frc.robot.subsystems.drivetrain.drivetrain;

public class RobotContainer {

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final XboxController driverController = new XboxController(Constants.OperatorConstants.driverDriveTrainPort);
  private final XboxController mechanismController = new XboxController(Constants.OperatorConstants.driverMechanismsPort);

  private final drivetrain m_Drivetrain = new drivetrain();
  private final intake m_Intake = new intake(Constants.Intake.intakeID, Constants.Intake.intakeIndexID, Constants.Sensors.intakeIndex, Constants.Intake.rollerSpeed);

  private Trigger tankTrigger = new Trigger((()-> Math.abs(driverController.getRightTriggerAxis()) > 0.2))
                            .or(new Trigger((()-> Math.abs(driverController.getLeftTriggerAxis()) > 0.2)));

  private Trigger alignSpkTrigger = new Trigger(()-> driverController.getXButton());
  private Trigger alignShtTrigger = new Trigger(()-> driverController.getStartButton());

  //private Trigger intakeTrigger = new Trigger((()-> Math.abs(mechanismController.getRightTriggerAxis()) > 0.2));

  public void registerCommands(){

  }
  
  public RobotContainer() {
    
    registerCommands();

    autoChooser = AutoBuilder.buildAutoChooser("EZ AUTO");
    autoChooser.addOption("EZ AUTO", new PathPlannerAuto("EZ AUTO"));
    autoChooser.addOption("Test", new PathPlannerAuto("Test"));
    autoChooser.addOption("EZ PATH", new PathPlannerAuto("EZ PATH"));

    m_Drivetrain.setDefaultCommand(new drivetrainCommandDefault(m_Drivetrain, 
                                   ()-> -driverController.getLeftY(), 
                                   ()-> -driverController.getLeftX(),
                                   ()-> -driverController.getRightX(),
                                   ()-> driverController.getBButtonPressed(),
                                   ()-> driverController.getYButtonPressed()));
                              
    m_Intake.setDefaultCommand(new intakeCommandDefault(m_Intake, ()-> mechanismController.getRightTriggerAxis(), ()-> mechanismController.getLeftTriggerAxis()));
                        

    configureBindings();

    SmartDashboard.putData("Autonomo",autoChooser);
  }

  private void configureBindings() {

    tankTrigger.onTrue(new drivetrainCommandTank(m_Drivetrain, 
                                                 ()-> driverController.getRightTriggerAxis(), 
                                                 ()-> driverController.getLeftTriggerAxis(),
                                                 ()-> (Math.abs(driverController.getRightTriggerAxis()) > 0.2 
                                                    || Math.abs(driverController.getLeftTriggerAxis()) > 0.2)));

    alignSpkTrigger.onTrue(new drivetrainCommandAlignSpeaker(m_Drivetrain,
                                                             ()-> -driverController.getLeftY(),
                                                             ()-> -driverController.getLeftX(), 
                                                             ()-> driverController.getXButton()));

    alignShtTrigger.onTrue(new drivetrainCommandAlignShuttle(m_Drivetrain,
                                                             ()-> -driverController.getLeftY(),
                                                             ()-> -driverController.getLeftX(), 
                                                             ()-> driverController.getStartButton()));

    // intakeTrigger.onTrue(new intakeCommand(m_Intake,
    //                     ()-> mechanismController.getRightTriggerAxis()));
    
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return  autoChooser.getSelected();
  }
}
