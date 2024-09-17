package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.ModuleOptimizer;
import frc.robot.Constants;

public class swerveModule extends SubsystemBase {

    public int moduleID;

    private Rotation2d lastAngle;
    private Rotation2d angleOffset;
    private Rotation2d encoderOffset;

    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANcoder angleEncoder;

    //  private final SimpleMotorFeedforward feedforward =  new SimpleMotorFeedforward(Constants.Swerve.driveKS, 
    //                                                                                Constants.Swerve.driveKV, 
    //                                                                                Constants.Swerve.driveKA); 

    public swerveModule(int moduleID, SwerveModuleConstants moduleConstants){

        this.moduleID = moduleID;
        angleOffset = moduleConstants.angleOffset;
        encoderOffset = moduleConstants.encoderOffset;

        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(new MagnetSensorConfigs()
                                                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                                                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    

        steerMotor = new TalonFX(moduleConstants.angleMotorID);
        steerMotor.getConfigurator().apply(new TalonFXConfiguration());
        steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit));
        steerMotor.setControl(new StaticBrake());
        steerMotor.setInverted(Constants.Swerve.angleInvert);

        var slot0Configs1 = new Slot0Configs();
        slot0Configs1.kP = Constants.Swerve.angleKP;
        slot0Configs1.kI = Constants.Swerve.angleKI;
        slot0Configs1.kD = Constants.Swerve.angleKD;

        steerMotor.getConfigurator().apply(slot0Configs1);

        resetToAbsolute();

        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit));
        driveMotor.setControl(new StaticBrake());
        driveMotor.setInverted(Constants.Swerve.driveInvert);

        var slot0Configs2 = new Slot0Configs();
        slot0Configs2.kP = Constants.Swerve.driveKP;
        slot0Configs2.kI = Constants.Swerve.driveKI;
        slot0Configs2.kD = Constants.Swerve.driveKD;

        driveMotor.getConfigurator().apply(slot0Configs2);

        lastAngle = getState().angle;

    }

    public void resetToAbsolute(){
        steerMotor.setPosition(0.0);

        double absolutePosition = getCANCoderAngle().getDegrees() - angleOffset.getDegrees();
        steerMotor.setPosition(absolutePosition);

    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        desiredState = ModuleOptimizer.optimize(desiredState, getState().angle);
    
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setAngle(SwerveModuleState desiredState) {

        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.1))
                ? lastAngle
                : desiredState.angle;
    
        steerMotor.setControl(new PositionDutyCycle(angle.getDegrees()));
        lastAngle = angle;

    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {

            double power = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(power);
        
        } 
        else {

            driveMotor.setControl(new VelocityDutyCycle(desiredState.speedMetersPerSecond));
              
        }
    }
    public double getDrivePosition(){

        return driveMotor.getPosition().getValue() * Constants.Swerve.driveConversionPositionFactor;

    }
    public double getDriveVelocity(){

        return driveMotor.getVelocity().getValue() * Constants.Swerve.driveConversionVelocityFactor;
    }


    public Rotation2d getSteerAngle() {

        double angle = steerMotor.getPosition().getValue() * Constants.Swerve.angleConversionFactor;
        return Rotation2d.fromDegrees(angle);

    }


    public SwerveModulePosition getPosition() {

        return new SwerveModulePosition(getDrivePosition(), getSteerAngle());

    }

    public SwerveModuleState getState() {

        return new SwerveModuleState(getDriveVelocity(), getSteerAngle());

    }

    public SwerveModuleState getStateEncoder() {

        return new SwerveModuleState(getDriveVelocity(), getCANCoderAngle());

    }

    public Rotation2d getCANCoderAngle() {


        double angle = ((angleEncoder.getAbsolutePosition().getValue()) * 360.0) - encoderOffset.getDegrees();
        return Rotation2d.fromDegrees(angle);

    }
    
}
