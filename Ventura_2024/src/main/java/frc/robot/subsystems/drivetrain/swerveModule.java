package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.CTREConfig;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.Conversions;
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

    private final SimpleMotorFeedforward feedforward =  new SimpleMotorFeedforward(Constants.Swerve.driveKS, 
                                                                                   Constants.Swerve.driveKV, 
                                                                                   Constants.Swerve.driveKA);

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityDutyCycle driveVelocity = new VelocityDutyCycle(0);

    private final PositionDutyCycle anglePosition = new PositionDutyCycle(0);

    public swerveModule(int moduleID, SwerveModuleConstants moduleConstants){

        this.moduleID = moduleID;
        this.angleOffset = moduleConstants.angleOffset;
        this.encoderOffset = moduleConstants.encoderOffset;

        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1).withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
        
        steerMotor = new TalonFX(moduleConstants.angleMotorID);
        var steer0Configs = new Slot0Configs();
        steer0Configs.kP = Constants.Swerve.angleKP;
        steer0Configs.kI = Constants.Swerve.angleKI;
        steer0Configs.kD = Constants.Swerve.angleKD; 
        steerMotor.getConfigurator().apply(steer0Configs);
        steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit));
        steerMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive));
        steerMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(Constants.Swerve.angleGearRatio));
        resetToAbsolute();

        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        var drive0Configs = new Slot0Configs();
        drive0Configs.kP = Constants.Swerve.driveKP;
        drive0Configs.kI = Constants.Swerve.driveKI;
        drive0Configs.kD = Constants.Swerve.driveKD;
        driveMotor.getConfigurator().apply(drive0Configs);
        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit));
        driveMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.CounterClockwise_Positive));
        driveMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(Constants.Swerve.driveGearRatio));
        driveMotor.setPosition(0.0);


    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        desiredState = ModuleOptimizer.optimize(desiredState, getState().angle);
    
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setAngle(SwerveModuleState desiredState) {

        // Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        //         ? lastAngle
        //         : desiredState.angle;
    
        steerMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        //lastAngle = angle;

    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {

            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.setControl(driveDutyCycle);
        
        } 
        else {

            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = feedforward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocity);
              
        }
    }

    public Rotation2d getCANCoderAngle() {


        double angle = ((angleEncoder.getAbsolutePosition().getValue())) - encoderOffset.getRotations();
        return Rotation2d.fromRotations(angle);

    }

    public void resetToAbsolute(){

        double absolutePosition = getCANCoderAngle().getRotations() - angleOffset.getRotations();
        steerMotor.setPosition(absolutePosition);

    }
    
    public double getDrivePosition(){

        return Conversions.rotationsToMeters(driveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference);

    }
    public double getDriveVelocity(){

        return Conversions.RPSToMPS(driveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference);
    }


    public Rotation2d getSteerAngle() {

        double angle = steerMotor.getPosition().getValue();
        return Rotation2d.fromRotations(angle);

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

}
