package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.handler;
import frc.robot.subsystems.shooter;

public class shooterCommandDefault  extends Command{

    private shooter shooter;
    private handler handler;
    private int state = 0;
    private boolean setShoot;
    private double startTime, shooterPos, indexSpeed;
    private Supplier<Boolean> aButton, bButton, rBumper, lBumper;
    private Supplier<Double> rTrigger;

    public shooterCommandDefault(shooter shooter, handler handler,Supplier<Boolean> aButton, Supplier<Boolean> bButton, Supplier<Boolean> rBumper, Supplier<Boolean> lBumper, Supplier<Double> rTrigger){

        this.shooter = shooter;
        this.handler = handler;
        this.rBumper = rBumper;
        this.lBumper = lBumper;
        this.aButton = aButton;
        this.bButton = bButton;
        this.rTrigger = rTrigger;

        addRequirements(shooter);

    }

    @Override
    public void initialize(){

        indexSpeed = 0.0;
        shooterPos = 0.46;
        startTime = 0.0;
        setShoot = false;
        state = 0;

    }

    @Override
    public void execute(){

        
        if(!aButton.get()) {

            shooterPos = 0.46;
            indexSpeed = 0.0;
            state = 0;
            startTime = 0.0;
            setShoot = false;
            shooter.stopShooter();
    

            if(bButton.get()) {

                indexSpeed = -1.0;

            }
            if(handler.getCanClimb()) {

                shooterPos = 0.58;
                setShoot = false;
                state = 0;

            }

        }

        else {


            if(handler.getCanShoot()) {


                Optional<Alliance> ally = DriverStation.getAlliance();
                double xPos;
                double yPos;


        
                if (ally.isPresent()) {
                    if (ally.get() == Alliance.Red) {
                        xPos = 16.8 - handler.getRobotPose().getX();
                        yPos = handler.getRobotPose().getY();                   
                    }

                    else if (ally.get() == Alliance.Blue) {
                        xPos = handler.getRobotPose().getX();
                        yPos = handler.getRobotPose().getY();
                    }
                    
                    else {
                        xPos = handler.getRobotPose().getX();
                        yPos = handler.getRobotPose().getY();
                    }
                }

                else {

                    xPos = handler.getRobotPose().getX();
                    yPos = handler.getRobotPose().getY();

                }


                shooterPos = handler.setRegression(0.58, -0.015, 0.003, xPos, yPos) + 0.0008;
                // System.out.print(setShoot);
                // System.out.println(state);
                setShoot = true;

                
            }

            else {

                setShoot = false;
                indexSpeed = 0.0;
                state = 0;
                startTime = 0.0;
                shooter.stopShooter();

            }

            if(setShoot == true){

                if(state == 0) {
                    shooter.setShooterVelocity(20);


                    if(rTrigger.get() > 0.2){

                        shooter.setShooterVelocity(80);
                        state++;

                    }
                }

                else if (state == 1){

                    if(rTrigger.get() < 0.2 && (Math.abs(shooter.getShooterVelocity()[0])  >= 70 || Math.abs(shooter.getShooterVelocity()[1])  >= 70)){

                        startTime = Timer.getFPGATimestamp();
                        state++;

                    }
                    

                }
                else if(state == 2){

                    indexSpeed = 1.0;

                    if(Timer.getFPGATimestamp() - startTime > 0.8){

                        indexSpeed = 0.0;
                        state++;

                    }

                }

                else if(state == 3){

                    System.out.println("AQUI");

                    handler.setCanIntake(true);
                    handler.setCanShoot(false);
                    setShoot = false;
                    shooterPos = 0.46;
                    indexSpeed = 0.0;
                    state = 0;
                    startTime = 0.0;

                }

            }
        }
        
        shooter.setShooterPosition(shooterPos);
        shooter.setIndexer(indexSpeed);
        //System.out.println(shooterPos);

        if(rBumper.get()){

            handler.setCanIntake(false);
            handler.setCanShoot(true);

        }
        

        if(lBumper.get()){

            handler.setCanIntake(true);
            handler.setCanShoot(false);

        }

    }

    @Override
    public void end(boolean interrupted){
    
        state = 0;
        startTime = 0;
        setShoot = false;

    }
    
    
}
