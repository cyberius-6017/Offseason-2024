package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class intake extends SubsystemBase {

    private TalonFX motorIntake;
    private TalonFX motorIndexIntake;
    private DigitalInput isNoteIn;
    private double rollerSpeed;
    private boolean rolling = false;
    private final VelocityDutyCycle intakeVelocity = new VelocityDutyCycle(0);
    // private final VelocityDutyCycle indexVelocity = new VelocityDutyCycle(0);

    public intake(int intakeID, int intakeIndexID, int sensorID, double rollerSpeed) {
        this.motorIntake = new TalonFX(intakeID);
        this.motorIntake = new TalonFX(intakeIndexID);
        this.isNoteIn = new DigitalInput(sensorID);
        this.rollerSpeed = rollerSpeed;
    }

    public void roll() {
        if (rolling) {
            if (!getNoteStatus()) {
                intakeVelocity.Velocity = rollerSpeed;
                motorIntake.setControl(intakeVelocity);
                motorIndexIntake.setControl(intakeVelocity);
            } else {
                intakeVelocity.Velocity = 0.0;
                motorIntake.setControl(intakeVelocity);
                motorIndexIntake.setControl(intakeVelocity);
            }
        } else {
            intakeVelocity.Velocity = 0.0;
            motorIntake.setControl(intakeVelocity);
            motorIndexIntake.setControl(intakeVelocity);
        }
    }

    public void startRoll() {
        rolling = true;
        roll();
    }
    public void stopRoll() {
        rolling = false;
        roll();
    }


    public boolean getNoteStatus() {
        return isNoteIn.get();
    }
}
