package frc.robot;

import frc.robot.subsystem.drivetrain.SwerveDriveChassis;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.Units;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;


public final class Constants {
    public static class RobotConstants {

        // PLACEHOLDER VALUES --> CORRECT 
        private static final boolean kInvertBotLeftSide = false;
        private static final boolean kInvertBotRightSide = true; 

        private static final double kOdometryUpdateRate = 1.0; 

        private static final double kDriveMotorGearRatio = 1.0; 
        private static final double kSteerMotorGearRatio = 1.0; // final output value for whole gear box assmb
        private static final double kDriveWheelRadius = 1.0; // meters 

        private static final int kPigeon2Id = 0; 
        
        private static final Slot0Configs kDriveMotorGains = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKS(0).withKV(0.124);

        private static final Slot0Configs kSteerMotorGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.5)
            .withKS(0.1).withKV(1.91).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        private static final SwerveModuleConstants.ClosedLoopOutputType kDriveMotorClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage; 
        private static final SwerveModuleConstants.ClosedLoopOutputType kSteerMotorClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage;
        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated; // built in esp 
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
        private static final TalonFXConfiguration kDriveMotorConfiguration = new TalonFXConfiguration(); // no current limitation implemented for drive 
        
        public static final LinearVelocity kMaxLinearSpeed = Units.MetersPerSecond.of(4.0); 
        public static final AngularVelocity kMaxRotationalSpeed = Units.RadiansPerSecond.of(2.0);
        public static final double kSwerveRequestDeadbandDrive = .10; 
        public static final double kSwerveRequestDeadbandSteer = .10; 
        
        private static final TalonFXConfiguration kSteerMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(40.0) // only allocating 40 amps for steer to save power. 
                    .withStatorCurrentLimitEnable(true)
            );

        private static final CANcoderConfiguration kEncoderConfiguration = new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive) 
            );
        
        private static final Pigeon2Configuration kPigeon2Configuration = new Pigeon2Configuration()
            .withMountPose(
                new MountPoseConfigs() // Adjust based on robot relative mounting of the pigeon
                    .withMountPosePitch(0.0)
                    .withMountPoseRoll(0.0)
                    .withMountPoseYaw(0.0)
            );

        public static final SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName("5144")
            .withPigeon2Id(kPigeon2Id)
            .withPigeon2Configs(kPigeon2Configuration);
            

        private static final double kFrictionCoefVoltage = 1.0; // voltage to overcome carpet friction and move 
        

        // creates instances of module specific 'SwerveModuleConstants' --> Commonalities across all modules
        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> 
        moduleConstantsCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ()
            .withDriveMotorGearRatio(kDriveMotorGearRatio)
            .withSteerMotorGearRatio(kSteerMotorGearRatio)
            .withWheelRadius(kDriveWheelRadius)
            .withDriveMotorGains(kDriveMotorGains)
            .withSteerMotorGains(kSteerMotorGains)
            .withDriveMotorClosedLoopOutput(kDriveMotorClosedLoopOutput)
            .withSteerMotorClosedLoopOutput(kSteerMotorClosedLoopOutput)
            .withSpeedAt12Volts(kMaxLinearSpeed)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withDriveMotorInitialConfigs(kDriveMotorConfiguration)
            .withSteerMotorInitialConfigs(kDriveMotorConfiguration)
            .withEncoderInitialConfigs(kEncoderConfiguration)
            .withDriveFrictionVoltage(kFrictionCoefVoltage)
            .withSteerFrictionVoltage(kFrictionCoefVoltage);

        // FRONT LEFT MODULE
        private static final int kFrontLeftDriveMotorId = 1;
        private static final int kFrontLeftSteerMotorId = 2; 
        private static final int kFrontLeftMagnetEncoderId = 3; 
        private static final Angle kFrontLeftEncoderOffset = Units.Rotations.of(0.0); 
        private static final boolean kFrontleftEncoderInversion = false; 
        private static final boolean kFrontLeftSteerInversion = true; 
        
        private static final Distance kFrontLeftModuleX = Units.Inches.of(0.0); 
        private static final Distance kFrontLeftModuleY = Units.Inches.of(0.0);
        
        // FRONT RIGHT MODULE
        private static final int kFrontRightDriveMotorId = 4;
        private static final int kFrontRightSteerMotorId = 5; 
        private static final int kFrontRightMagnetEncoderId = 6; 
        private static final Angle kFrontRightEncoderOffset = Units.Rotations.of(0.0); 
        private static final boolean kFrontRightEncoderInversion = false; 
        private static final boolean kFrontRightSteerInversion = true; 
        
        private static final Distance kFrontRightModuleX = Units.Inches.of(0.0); 
        private static final Distance kFrontRightModuleY = Units.Inches.of(0.0);

        // REAR LEFT MODULE
        private static final int kRearLeftDriveMotorId = 7;
        private static final int kRearLeftSteerMotorId = 8; 
        private static final int kRearLeftMagnetEncoderId = 9; 
        private static final Angle kRearLeftEncoderOffset = Units.Rotations.of(0.0); 
        private static final boolean kRearLeftEncoderInversion = true; 
        private static final boolean kRearLeftSteerInversion = false; 
        
        private static final Distance kRearLeftModuleX = Units.Inches.of(0.0); 
        private static final Distance kRearLeftModuleY = Units.Inches.of(0.0);

        // REAR RIGHT MODULE
        private static final int kRearRightDriveMotorId = 10;
        private static final int kRearRightSteerMotorId = 11; 
        private static final int kRearRightMagnetEncoderId = 12; 
        private static final Angle kRearRightEncoderOffset = Units.Rotations.of(0.0); 
        private static final boolean kRearRightEncoderInversion = true; 
        private static final boolean kRearRightSteerInversion = false; 
        
        private static final Distance kRearRightModuleX = Units.Inches.of(0.0); 
        private static final Distance kRearRightModuleY = Units.Inches.of(0.0);
      
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeftConfiguration = 
            moduleConstantsCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftMagnetEncoderId, kFrontLeftEncoderOffset, kFrontLeftModuleX,
                kFrontLeftModuleY, kInvertBotLeftSide, kFrontLeftSteerInversion, kFrontleftEncoderInversion
            );

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRightConfiguration = 
            moduleConstantsCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightMagnetEncoderId, kFrontRightEncoderOffset, kFrontRightModuleX,
                kFrontRightModuleY, kInvertBotRightSide, kFrontRightSteerInversion, kFrontRightEncoderInversion
            );

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> rearLeftConfiguration = 
            moduleConstantsCreator.createModuleConstants(
                kRearLeftSteerMotorId, kRearLeftDriveMotorId, kRearLeftMagnetEncoderId, kRearLeftEncoderOffset, kRearLeftModuleX,
                kRearLeftModuleY, kInvertBotLeftSide, kRearLeftSteerInversion, kRearLeftEncoderInversion
            );

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> rearRightConfiguration = 
            moduleConstantsCreator.createModuleConstants(
                kRearRightSteerMotorId, kRearRightDriveMotorId, kRearRightMagnetEncoderId, kRearRightEncoderOffset, kRearRightModuleX,
                kRearRightModuleY, kInvertBotRightSide, kRearRightSteerInversion, kRearRightEncoderInversion
            );
        
        public static SwerveDriveChassis createSwerveDriveChassis() {
            return new SwerveDriveChassis (
                drivetrainConstants, kOdometryUpdateRate, frontLeftConfiguration, frontRightConfiguration, rearLeftConfiguration, rearRightConfiguration
            );
        }
    }

      
    public static class OperatorConstants {
        public static final int kDriverPortId = 0; 
        public static final int kOperatorPortId = 1;
    }
    public static class FieldConstants {

    }
    public static class PathingConstants {

    }
}

