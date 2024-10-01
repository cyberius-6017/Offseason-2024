package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {

    private TalonFX IndexShooter;
    private TalonFX ShooterUp;
    private TalonFX ShooterDw;
    private double rollerIn, rollerOut;
    private VelocityDutyCycle shooterVelocity = new VelocityDutyCycle(0);


    public Shooter(int InShooterID, int ShootUpID, int ShootDwID, double rollerIn, double rollerOut) {
        this.IndexShooter = new TalonFX(InShooterID);
        this.ShooterUp = new TalonFX(ShootUpID);
        this.ShooterDw = new TalonFX(ShootDwID);
        this.rollerIn = rollerIn;
        this.rollerOut = rollerOut;
    }

    public void index() {
        shooterVelocity.Velocity = rollerIn;
        IndexShooter.setControl(shooterVelocity);
    }
    public void shoot() {
        shooterVelocity.Velocity = rollerOut;
        ShooterUp.setControl(shooterVelocity);
        ShooterDw.setControl(shooterVelocity);
    }
}
    