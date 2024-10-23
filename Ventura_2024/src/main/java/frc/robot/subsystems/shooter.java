package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

public class shooter extends SubsystemBase{

    private TalonFX shooterLeft;
    private TalonFX shooterRight;
    private TalonFX index;
    private TalonFX pivot;

    private LaserCan isNoteIn;    

    private CANcoder shooterPosition;

    private double encoderOffset;

    public shooter(int shooterLeftID, int shooterRightID, int indexID, int pivotID, int encoderID, double encoderOffset, int sensorID){

        this.encoderOffset = encoderOffset;

        isNoteIn = new LaserCan(sensorID);

        shooterPosition = new CANcoder(encoderID);
        shooterPosition.getConfigurator().apply(new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1).withSensorDirection(SensorDirectionValue.Clockwise_Positive));

        shooterLeft = new TalonFX(shooterLeftID);
        shooterRight = new TalonFX(shooterRightID);
        index = new TalonFX(indexID);
        pivot = new TalonFX(pivotID);

        var shooterLeft0Configs = new Slot0Configs();
        shooterLeft0Configs.kP = Constants.Shooter.shooterLKP;
        shooterLeft0Configs.kI = Constants.Shooter.shooterLKI;
        shooterLeft0Configs.kD = Constants.Shooter.shooterLKD;
        shooterLeft.getConfigurator().apply(shooterLeft0Configs);
        shooterLeft.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive));
        shooterLeft.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Shooter.shooterCurrentLImit));
        shooterLeft.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(Constants.Shooter.closedLoopRamp));

        var shooterRight0Configs = new Slot0Configs();
        shooterRight0Configs.kP = Constants.Shooter.shooterRKP;
        shooterRight0Configs.kI = Constants.Shooter.shooterRKI;
        shooterRight0Configs.kD = Constants.Shooter.shooterRKD;
        shooterRight.getConfigurator().apply(shooterRight0Configs);
        shooterRight.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive)); 
        shooterRight.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Shooter.shooterCurrentLImit));
        shooterRight.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(Constants.Shooter.closedLoopRamp));

        var index0Configs = new Slot0Configs(); 
        index0Configs.kP = Constants.Shooter.indexKP;
        index0Configs.kI = Constants.Shooter.indexKI;
        index0Configs.kD = Constants.Shooter.indexKD;
        index.getConfigurator().apply(index0Configs);
        index.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive));
        index.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Shooter.shooterCurrentLImit));
        index.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(Constants.Shooter.closedLoopRamp));

        var pivot0Configs = new Slot0Configs(); 
        pivot0Configs.kP = Constants.Shooter.pivotKP;
        pivot0Configs.kI = Constants.Shooter.pivotKI;
        pivot0Configs.kD = Constants.Shooter.pivotKD;
        pivot.getConfigurator().apply(pivot0Configs);
        pivot.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.CounterClockwise_Positive));        
        pivot.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Shooter.shooterCurrentLImit));
        pivot.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(Constants.Shooter.closedLoopRamp));
        pivot.getConfigurator().apply(new FeedbackConfigs().withFeedbackRemoteSensorID(encoderID).withRemoteCANcoder(shooterPosition));

    }

    public void setShooterVelocity(double vel){

        shooterLeft.setControl(new VelocityDutyCycle(-vel));
        shooterRight.setControl(new VelocityDutyCycle(-vel));


    }

    public double[] getShooterVelocity(){

        double[] velocities = {shooterLeft.getVelocity().getValueAsDouble(),
                               shooterRight.getVelocity().getValueAsDouble()};

        return velocities;

    }

    public boolean getNoteStatus(){

        LaserCan.Measurement measurement = isNoteIn.getMeasurement();

        // if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        //     return true;
        // } else {
        //   return false;
        // }

        if(getNoteDistance() < 60 && getNoteDistance() > 10){

            return true;

        }
        else{

            return false;
        }

    }

    public double getNoteDistance(){

        LaserCan.Measurement measurement = isNoteIn.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return(measurement.distance_mm);
        } else {
          return 0.0;
        }

    }

    

    public void stopShooter(){

        shooterLeft.stopMotor();
        shooterRight.stopMotor();

    }

    public void setIndexer(double speed){

        index.set(speed);

    }

    public void setShooterPosition(double pos){

        pivot.setControl(new PositionDutyCycle(pos));

    }

    public void setShooterPositionDegrees(double degrees){

        pivot.setControl(new PositionDutyCycle(degrees / 360));

    }

    public double getShooterAbsolutePosition(){

        return shooterPosition.getAbsolutePosition().getValueAsDouble() - encoderOffset;

    }

    public double getShooterPositionDegrees(){

        return getShooterAbsolutePosition() * 360;

    }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("Note Dis: ", getNoteDistance());
        SmartDashboard.putBoolean("Note: ", getNoteStatus());
        SmartDashboard.putNumber("Shooter Pos: ", getShooterAbsolutePosition());
        SmartDashboard.putNumberArray("Shooter Vel", getShooterVelocity());
        SmartDashboard.putBoolean("Limit Fwd", (pivot.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround));
        SmartDashboard.putBoolean("Limit Rev", (pivot.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround));
        

    }
    
}
