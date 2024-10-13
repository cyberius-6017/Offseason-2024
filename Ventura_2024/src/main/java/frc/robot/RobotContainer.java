package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arduinoCommunicationCommand;
import frc.robot.commands.climberCommand;
import frc.robot.commands.climberCommandDefault;
import frc.robot.commands.drivetrainCommandAlignShuttle;
import frc.robot.commands.drivetrainCommandAlignSpeaker;
import frc.robot.commands.drivetrainCommandDefault;
import frc.robot.commands.drivetrainCommandTank;
import frc.robot.commands.intakeCommand;
import frc.robot.commands.intakeCommandAuto;
import frc.robot.commands.intakeCommandDefault;
import frc.robot.commands.shooterCommand;
import frc.robot.commands.shooterCommandAuto;
import frc.robot.commands.shooterCommandAutoInit;
import frc.robot.commands.shooterCommandDefault;
import frc.robot.commands.shooterCommandLowPassNote;
import frc.robot.commands.shooterCommandPassNote;
import frc.robot.commands.shooterCommandShootNote;
import frc.robot.subsystems.arduinoComm;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.handler;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.shooter;
// import frc.robot.commands.drivetrainCommand;
// import frc.robot.commands.drivetrainTankCommand;
import frc.robot.subsystems.drivetrain.drivetrain;

public class RobotContainer {

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final XboxController driverController = new XboxController(Constants.OperatorConstants.driverDriveTrainPort);
  private final XboxController mechanismController = new XboxController(Constants.OperatorConstants.driverMechanismsPort);

  private final drivetrain m_Drivetrain = new drivetrain();
  // private final arduinoComm m_ArduinoComm = new arduinoComm();
  private final intake m_Intake = new intake(Constants.Intake.intakeID, 
                                             Constants.Intake.intakeIndexID, 
                                             Constants.Sensors.intakeIndex, 
                                             Constants.Intake.rollerSpeed);
  
  private final shooter m_Shooter = new shooter(Constants.Shooter.shooterLeft, 
                                                Constants.Shooter.shooterRight, 
                                                Constants.Shooter.indexID, 
                                                Constants.Shooter.pivotID, 
                                                Constants.Shooter.encoderID, 
                                                Constants.Shooter.encoderOffset,
                                                Constants.Sensors.shooterSensor);
  private final handler m_Handler = new handler();

  private final climber m_Climber = new climber(Constants.Climber.climberLeftID, Constants.Climber.climberRightID);

  private Trigger tankTrigger = new Trigger((()-> Math.abs(driverController.getRightTriggerAxis()) > 0.2))
                            .or(new Trigger((()-> Math.abs(driverController.getLeftTriggerAxis()) > 0.2)));

  private Trigger alignSpkTrigger = new Trigger(()-> driverController.getXButton());
  private Trigger alignShtTrigger = new Trigger(()-> driverController.getAButton());

  private Trigger intakeTrigger = new Trigger((()-> Math.abs(mechanismController.getLeftTriggerAxis()) > 0.2))
                             .and(new Trigger(()-> handler.canIntake));
  private Trigger shooterPassTrigger = new Trigger((()-> mechanismController.getPOV() == 0))
                              .and(new Trigger(()-> handler.canShoot));
  private Trigger shooterLowPassTrigger = new Trigger((()-> mechanismController.getPOV() == 180))
                              .and(new Trigger(()-> handler.canShoot));
  private Trigger shooterShootTrigger = new Trigger((()-> Math.abs(mechanismController.getRightTriggerAxis()) > 0.2))
                              .and(new Trigger(()-> handler.canShoot));

  private Trigger abortTrigger = new Trigger(()-> mechanismController.getStartButtonPressed());

  private Trigger climbTrigger = new Trigger(()-> mechanismController.getRightStickButtonPressed() && mechanismController.getLeftStickButtonPressed());

  public void registerCommands(){

    NamedCommands.registerCommand("Shooter Auto Init", new shooterCommandAutoInit(m_Shooter));
    NamedCommands.registerCommand("Shooter Auto", new shooterCommandAuto(m_Shooter));
    NamedCommands.registerCommand("Intake Auto", new intakeCommandAuto(m_Intake, m_Shooter));

  }
  
  public RobotContainer() {
    
    registerCommands();

    autoChooser = AutoBuilder.buildAutoChooser("EZ AUTO");
    autoChooser.addOption("EZ AUTO", new PathPlannerAuto("EZ AUTO"));
    autoChooser.addOption("Test", new PathPlannerAuto("Test"));
    autoChooser.addOption("EZ PATH", new PathPlannerAuto("EZ PATH"));
    autoChooser.addOption("No se", new PathPlannerAuto("No se Auto"));

    m_Drivetrain.setDefaultCommand(new drivetrainCommandDefault(m_Drivetrain, 
                                  m_Handler,
                                   ()-> -driverController.getLeftY(), 
                                   ()-> -driverController.getLeftX(),
                                   ()-> -driverController.getRightX(),
                                   ()-> driverController.getBButtonPressed(),
                                   ()-> driverController.getYButtonPressed()));
                              
    m_Intake.setDefaultCommand(new intakeCommandDefault(m_Intake,
                                                        ()-> mechanismController.getBButton()));

    m_Shooter.setDefaultCommand(new shooterCommandDefault(m_Shooter, 
                                                          m_Handler,
                                                          ()-> mechanismController.getRightBumperPressed(),
                                                          ()-> mechanismController.getLeftBumperPressed()));

    // m_ArduinoComm.setDefaultCommand(new arduinoCommunicationCommand(m_ArduinoComm,
    //                                                                 m_Handler));

    m_Climber.setDefaultCommand(new climberCommand(m_Climber,
                                                  m_Handler,
                                                  m_Shooter));
    
    m_Climber.setDefaultCommand(new climberCommandDefault(m_Climber,
                                                          m_Handler,
                                                          ()-> mechanismController.getXButtonPressed(),
                                                          ()-> mechanismController.getYButton(),
                                                          ()-> mechanismController.getLeftY(),
                                                          ()-> mechanismController.getRightY()));
                        

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
                                                             m_Handler,
                                                             ()-> -driverController.getLeftY(),
                                                             ()-> -driverController.getLeftX(), 
                                                             ()-> driverController.getXButton()));

    alignShtTrigger.onTrue(new drivetrainCommandAlignShuttle(m_Drivetrain,
                                                             ()-> -driverController.getLeftY(),
                                                             ()-> -driverController.getLeftX(), 
                                                             ()-> driverController.getStartButton()));

    intakeTrigger.onTrue(new intakeCommand(m_Intake,
                                           m_Shooter,
                                           m_Handler));
    
    shooterPassTrigger.onTrue(new shooterCommandPassNote(m_Shooter, 
                                             m_Handler, 
                                             ()-> mechanismController.getPOV() == 0));

    shooterLowPassTrigger.onTrue(new shooterCommandLowPassNote(m_Shooter, 
                                             m_Handler, 
                                             ()-> mechanismController.getPOV() == 180));

    shooterShootTrigger.onTrue(new shooterCommandShootNote(m_Shooter, 
                                             m_Handler, 
                                             ()-> mechanismController.getLeftTriggerAxis() > 0.2));

    abortTrigger.onTrue(new shooterCommandDefault(m_Shooter, 
                                                  m_Handler,
                                                  ()-> mechanismController.getRightBumperPressed(),
                                                  ()-> mechanismController.getLeftBumperPressed())
             .alongWith(new intakeCommandDefault(m_Intake,
                                                 ()-> mechanismController.getBButton()))
             .alongWith(new climberCommandDefault(m_Climber,
                                                  m_Handler,
                                                  ()-> mechanismController.getXButtonPressed(),
                                                  ()-> mechanismController.getYButton(),
                                                  ()-> mechanismController.getLeftY(),
                                                  ()-> mechanismController.getRightY())));
    climbTrigger.onTrue(new climberCommand(m_Climber, 
                                             m_Handler,
                                             m_Shooter));

  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return  autoChooser.getSelected();
  }

  
}
