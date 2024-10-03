package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class intake extends SubsystemBase {

    private TalonFX motorIntake;
    private TalonFX motorIndex;
    private DigitalInput isNoteIn;
    private double rollerSpeed;
    private boolean rolling = false;
    private int intakeID;
    private int intakeIndexID;
    private final VelocityDutyCycle intakeVelocity = new VelocityDutyCycle(0);
    private final VelocityDutyCycle indexVelocity = new VelocityDutyCycle(0);

    public intake(int intakeID, int intakeIndexID, int sensorID, double rollerSpeed) {
        this.intakeID = intakeID;
        this.intakeIndexID = intakeIndexID;
        this.isNoteIn = new DigitalInput(sensorID);
        this.rollerSpeed = rollerSpeed;

        motorIntake = new TalonFX(intakeID);
        motorIndex = new TalonFX(intakeIndexID);

        var intake0Configs = new Slot0Configs();
        intake0Configs.kP = Constants.Intake.intakeKP;
        intake0Configs.kI = Constants.Intake.intakeKI;
        intake0Configs.kD = Constants.Intake.intakeKD;
        motorIntake.getConfigurator().apply(intake0Configs); 
        motorIntake.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive));
        motorIntake.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Intake.intakeCurrentLimit));
        motorIntake.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(Constants.Intake.closedLoopRamp));
        
        var index0Configs = new Slot0Configs();
        index0Configs.kP = Constants.Intake.indexKP;
        index0Configs.kI = Constants.Intake.indexKI;
        index0Configs.kD = Constants.Intake.indexKD;
        motorIndex.getConfigurator().apply(index0Configs); 
        motorIndex.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        motorIndex.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Intake.intakeCurrentLimit));
        motorIndex.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(Constants.Intake.closedLoopRamp));


    }

    public void roll() {
        intakeVelocity.Velocity = rollerSpeed;
        motorIntake.setControl(intakeVelocity);
        motorIndex.setControl(intakeVelocity);
    }
    public void setRoller(double speed){
  
        motorIntake.set(-speed);
        motorIndex.set(-speed);

    }

    public void stopRoller(){

        motorIntake.stopMotor();
        motorIndex.stopMotor();

    }

    public void setIntakeVelocity(double vel){

        motorIntake.setControl(intakeVelocity.withVelocity(vel));
        motorIndex.setControl(indexVelocity.withVelocity(vel));

    }

    public boolean getNoteStatus() {

        return isNoteIn.get();
    
    }
    
    public double getIntakeVelocity(){

        return motorIntake.getVelocity().getValueAsDouble();

    }

    public double getIndexVelocity(){

        return motorIndex.getVelocity().getValueAsDouble();

    }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("Intake Vel: ", getIntakeVelocity());
        SmartDashboard.putNumber("Index Vel:", getIndexVelocity());

    }
}
