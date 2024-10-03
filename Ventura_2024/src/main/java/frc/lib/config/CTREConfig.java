package frc.lib.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public final class CTREConfig {

    public static TalonFXConfiguration swerveSteerConfig = new TalonFXConfiguration();
    public static TalonFXConfiguration swerveDriveConfig = new TalonFXConfiguration();
    public static CANcoderConfiguration swerveCANCoderConfig = new CANcoderConfiguration();

    public CTREConfig(){

        swerveCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        swerveCANCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        swerveSteerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        swerveSteerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        swerveSteerConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        swerveSteerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        swerveSteerConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;

        swerveSteerConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveSteerConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveSteerConfig.Slot0.kD = Constants.Swerve.angleKD;

        swerveDriveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        swerveDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        swerveDriveConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        swerveDriveConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.driveContinuousCurrentLimit;

        swerveDriveConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveConfig.Slot0.kD = Constants.Swerve.driveKD;
        
        swerveDriveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;

    }

    
}
