package team3647.frc2023.constants;

import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import team3647.lib.SwerveModule;

public class SwerveDriveConstants {
        // default falcon rotates counter clockwise (CCW)
        // make sure gyro -CW, +CCW
        public static final boolean canCoderInvert = false;
        public static final boolean kDriveMotorInverted = false;
        public static final boolean kTurnMotorInverted = true;

        // physical possible max speed
        public static final double kDrivePossibleMaxSpeedMPS = 5;
        public static final double kRotPossibleMaxSpeedRadPerSec = 10;

        // public static final NeutralMode kTurnNeutralMode = NeutralMode.Coast;
        // public static final NeutralMode kDriveNeutralMode = NeutralMode.Brake;

        public static final TalonFX kFrontLeftDrive = new TalonFX(GlobalConstants.SwerveDriveIds.kFrontLeftDriveId,
                        "drive");
        public static final TalonFX kFrontLeftTurn = new TalonFX(GlobalConstants.SwerveDriveIds.kFrontLeftTurnId,
                        "drive");
        public static final CANcoder kFrontLeftAbsEncoder = new CANcoder(
                        GlobalConstants.SwerveDriveIds.kFrontLeftAbsEncoderPort, "drive");

        public static final TalonFX kFrontRightDrive = new TalonFX(GlobalConstants.SwerveDriveIds.kFrontRightDriveId,
                        "drive");
        public static final TalonFX kFrontRightTurn = new TalonFX(GlobalConstants.SwerveDriveIds.kFrontRightTurnId,
                        "drive");
        public static final CANcoder kFrontRightAbsEncoder = new CANcoder(
                        GlobalConstants.SwerveDriveIds.kFrontRightAbsEncoderPort, "drive");

        public static final TalonFX kBackLeftDrive = new TalonFX(GlobalConstants.SwerveDriveIds.kBackLeftDriveId,
                        "drive");
        public static final TalonFX kBackLeftTurn = new TalonFX(GlobalConstants.SwerveDriveIds.kBackLeftTurnId,
                        "drive");
        public static final CANcoder kBackLeftAbsEncoder = new CANcoder(
                        GlobalConstants.SwerveDriveIds.kBackLeftAbsEncoderPort, "drive");

        public static final TalonFX kBackRightDrive = new TalonFX(GlobalConstants.SwerveDriveIds.kBackRightDriveId,
                        "drive");
        public static final TalonFX kBackRightTurn = new TalonFX(GlobalConstants.SwerveDriveIds.kBackRightTurnId,
                        "drive");
        public static final CANcoder kBackRightAbsEncoder = new CANcoder(
                        GlobalConstants.SwerveDriveIds.kBackRightAbsEncoderPort, "drive");

        public static final Pigeon2Configuration kGyroConfig = new Pigeon2Configuration();

        // config swerve module reversed here, module class doens't reverse for you

        // distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(20.75);
        // distance between front and back wheels

        public static final double kWheelBase = Units.inchesToMeters(20.75);
        // translations are locations of each module wheel
        // 0 --> ++ --> front left
        // 1 --> +- --> front right
        // 2 --> -+ --> back left
        // 3 --> -- --> back right
        // c is center of robot,
        // +x towards front of robot, +y towards left of robot
        // +x
        // ^
        // |
        // +y<--c
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

        // config conversion factors here for each module. in meters for postiion and
        // radians for
        // rotation.

        // from motor to output shaft
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurnMotorGearRatio = 7.0 / 150.0;
        public static final double kWheelDiameterMeters = 0.097; // 97mm

        // // divide for tick to deg
        public static final double kTurnMotorNativeToDeg = kTurnMotorGearRatio / GlobalConstants.kFalconTicksPerRotation
                        * 360.0;

        public static final double kTurnMotorNativeToDPS = kTurnMotorNativeToDeg * 10.0; // RPS / Native/10ms

        public static final double kWheelRotationToMetersDrive = kWheelDiameterMeters * Math.PI * kDriveMotorGearRatio;

        // Multiply by 10 because velocity is in ticks/100ms
        public static final double kFalconVelocityToMpS = kWheelRotationToMetersDrive * 10.0
                        / GlobalConstants.kFalconTicksPerRotation;

        public static final double kFalconTicksToMeters = kWheelRotationToMetersDrive
                        / GlobalConstants.kFalconTicksPerRotation;

        public static final double kNominalVoltage = 10;
        public static final double kStallCurrent = 35;
        public static final double kMaxCurrent = 60;

        // prac bot
        // public static final double kAbsFrontLeftEncoderOffsetDeg = 302.52;
        // public static final double kAbsFrontRightEncoderOffsetDeg = 244.77;
        // public static final double kAbsBackLeftEncoderOffsetDeg = 121.9;
        // public static final double kAbsBackRightEncoderOffsetDeg = 240.3;

        // comp bot
        // public static final double kAbsFrontLeftEncoderOffsetDeg = 37.01;
        // public static final double kAbsFrontRightEncoderOffsetDeg = 184.48;
        // public static final double kAbsBackLeftEncoderOffsetDeg = 348.13;
        // public static final double kAbsBackRightEncoderOffsetDeg = 246.88;

        // comp bot
        public static final double kAbsFrontLeftEncoderOffsetDeg = 35.595;
        public static final double kAbsFrontRightEncoderOffsetDeg = 182.724;
        public static final double kAbsBackLeftEncoderOffsetDeg = 348.222;
        public static final double kAbsBackRightEncoderOffsetDeg = 247.851;

        // max speed limits that we want
        public static final double kTeleopDriveMaxAccelUnitsPerSec = kDrivePossibleMaxSpeedMPS / 2;
        public static final double kTeleopDriveMaxAngularAccelUnitsPerSec = kRotPossibleMaxSpeedRadPerSec / 3;

        public static final TalonFXConfiguration kFrontLeftDriveConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kFrontLeftTurnConfig = new TalonFXConfiguration();

        public static final TalonFXConfiguration kFrontRightDriveConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kFrontRightTurnConfig = new TalonFXConfiguration();

        public static final TalonFXConfiguration kBackLeftDriveConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kBackLeftTurnConfig = new TalonFXConfiguration();

        public static final TalonFXConfiguration kBackRightDriveConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kBackRightTurnConfig = new TalonFXConfiguration();

        public static final CANcoderConfiguration kFrontLeftAbsConfig = new CANcoderConfiguration();
        public static final CANcoderConfiguration kFrontRightAbsConfig = new CANcoderConfiguration();
        public static final CANcoderConfiguration kBackLeftAbsConfig = new CANcoderConfiguration();
        public static final CANcoderConfiguration kBackRightAbsConfig = new CANcoderConfiguration();

        // master FF for drive for all modules
        public static final double kS = (0.56744 / 12); // 0.56744; // Volts
        public static final double kV = (2.5 / 12.0); // Volts
        public static final double kA = (0.0 / 12); // Volts

        public static final SimpleMotorFeedforward kMasterDriveFeedforward = new SimpleMotorFeedforward(kS, kV, kA);

        // master PID constants for turn and drive for all modules
        public static final double kDriveP = 0.00014;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;

        public static final double kTurnP = 0.4;
        public static final double kTurnI = 0.0;
        public static final double kTurnD = 0;

        public static final double kYP = 1;
        public static final double kYI = 0.0;
        public static final double kYD = 0;

        public static final PIDController kYController = new PIDController(kYP, kYI, kYD);

        public static final PIDController kAutoSteerXYPIDController = new PIDController(0.05, 0, 0);
        // 3*Pi = move at 10 rads per second if we are 180* away from target heading
        public static final PIDController kAutoSteerHeadingController = new PIDController(0.04, 0, 0);
        // PID constants for roll and yaw

        // is stored as reference?
        public static final SwerveModule kFrontLeftModule = new SwerveModule(
                        kFrontLeftDrive,
                        kFrontLeftTurn,
                        kMasterDriveFeedforward,
                        kFrontLeftAbsEncoder,
                        kAbsFrontLeftEncoderOffsetDeg,
                        kFalconVelocityToMpS,
                        kFalconTicksToMeters,
                        kTurnMotorNativeToDPS,
                        kTurnMotorNativeToDeg,
                        kNominalVoltage);
        public static final SwerveModule kFrontRightModule = new SwerveModule(
                        kFrontRightDrive,
                        kFrontRightTurn,
                        kMasterDriveFeedforward,
                        kFrontRightAbsEncoder,
                        kAbsFrontRightEncoderOffsetDeg,
                        kFalconVelocityToMpS,
                        kFalconTicksToMeters,
                        kTurnMotorNativeToDPS,
                        kTurnMotorNativeToDeg,
                        kNominalVoltage);
        public static final SwerveModule kBackLeftModule = new SwerveModule(
                        kBackLeftDrive,
                        kBackLeftTurn,
                        kMasterDriveFeedforward,
                        kBackLeftAbsEncoder,
                        kAbsBackLeftEncoderOffsetDeg,
                        kFalconVelocityToMpS,
                        kFalconTicksToMeters,
                        kTurnMotorNativeToDPS,
                        kTurnMotorNativeToDeg,
                        kNominalVoltage);
        public static final SwerveModule kBackRightModule = new SwerveModule(
                        kBackRightDrive,
                        kBackRightTurn,
                        kMasterDriveFeedforward,
                        kBackRightAbsEncoder,
                        kAbsBackRightEncoderOffsetDeg,
                        kFalconVelocityToMpS,
                        kFalconTicksToMeters,
                        kTurnMotorNativeToDPS,
                        kTurnMotorNativeToDeg,
                        kNominalVoltage);

        public static final Pigeon2 kGyro = new Pigeon2(GlobalConstants.SwerveDriveIds.gyroPin, "drive");

        private static void setTurnMotorConfig(TalonFXConfiguration config) {
                config.Slot0.kP = kTurnP;
                config.Slot0.kI = kTurnI;
                config.Slot0.kD = kTurnD;
                config.CurrentLimits.SupplyCurrentLimitEnable = true;
                config.CurrentLimits.SupplyCurrentThreshold = kMaxCurrent;
                config.CurrentLimits.SupplyTimeThreshold = 5;
                // config.voltageCompSaturation = kNominalVoltage;
                // config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        }

        private static void setDriveMotorConfig(TalonFXConfiguration config) {
                config.Slot0.kP = kDriveP;
                config.Slot0.kI = kDriveI;
                config.Slot0.kD = kDriveD;
                config.CurrentLimits.SupplyCurrentLimitEnable = false;
                config.CurrentLimits.SupplyCurrentThreshold = kMaxCurrent;
                config.CurrentLimits.SupplyTimeThreshold = 5;
        //         config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        //         config.voltageCompSaturation = kNominalVoltage;
        // }

        private static void printError(StatusCode error) {
                if (error.value == 0) {
                        return;
                }

                System.out.println(error);
        }

        static {
                kGyroConfig. = 0.3;//FIGURE THIS OUT
                // remove later
                kGyroConfig.MountPose.MountPoseYaw = 90;
                printError(kGyro.getConfigurator().apply(kGyroConfig, GlobalConstants.kTimeoutMS));

                setTurnMotorConfig(kFrontLeftTurnConfig);
                setTurnMotorConfig(kFrontRightTurnConfig);
                setTurnMotorConfig(kBackLeftTurnConfig);
                setTurnMotorConfig(kBackRightTurnConfig);

                setDriveMotorConfig(kFrontLeftDriveConfig);
                setDriveMotorConfig(kFrontRightDriveConfig);
                setDriveMotorConfig(kBackLeftDriveConfig);
                setDriveMotorConfig(kBackRightDriveConfig);

                kFrontLeftAbsConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
                kFrontLeftAbsConfig.sensorDirection = canCoderInvert;
                kFrontLeftAbsConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
                kFrontLeftAbsConfig.sensorTimeBase = SensorTimeBase.PerSecond;
                kFrontLeftAbsConfig.magnetOffsetDegrees = 0.0;

                kFrontRightAbsConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
                kFrontRightAbsConfig.sensorDirection = canCoderInvert;
                kFrontRightAbsConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
                kFrontRightAbsConfig.sensorTimeBase = SensorTimeBase.PerSecond;
                kFrontRightAbsConfig.magnetOffsetDegrees = 0.0;

                kBackLeftAbsConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
                kBackLeftAbsConfig.sensorDirection = canCoderInvert;
                kBackLeftAbsConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
                kBackLeftAbsConfig.sensorTimeBase = SensorTimeBase.PerSecond;
                kBackLeftAbsConfig.magnetOffsetDegrees = 0.0;
                kBackLeftAbsConfig.sensorCoefficient = 0;

                kBackRightAbsConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
                kBackRightAbsConfig.sensorDirection = canCoderInvert;
                kBackRightAbsConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
                kBackRightAbsConfig.sensorTimeBase = SensorTimeBase.PerSecond;
                kBackRightAbsConfig.magnetOffsetDegrees = 0.0;

                printError(
                                kFrontLeftAbsEncoder.configAllSettings(
                                                kFrontLeftAbsConfig, GlobalConstants.kTimeoutMS));
                printError(
                                kFrontRightAbsEncoder.configAllSettings(
                                                kFrontRightAbsConfig, GlobalConstants.kTimeoutMS));
                printError(
                                kBackLeftAbsEncoder.configAllSettings(
                                                kBackLeftAbsConfig, GlobalConstants.kTimeoutMS));
                printError(
                                kBackRightAbsEncoder.configAllSettings(
                                                kBackRightAbsConfig, GlobalConstants.kTimeoutMS));

                printError(
                                kFrontLeftTurn.configAllSettings(kFrontLeftTurnConfig, GlobalConstants.kTimeoutMS));
                kFrontLeftTurn.setNeutralMode(kTurnNeutralMode);
                printError(
                                kFrontRightTurn.configAllSettings(
                                                kFrontRightTurnConfig, GlobalConstants.kTimeoutMS));
                kFrontRightTurn.setNeutralMode(kTurnNeutralMode);
                printError(
                                kBackLeftTurn.configAllSettings(kBackLeftTurnConfig, GlobalConstants.kTimeoutMS));
                kBackLeftTurn.setNeutralMode(kTurnNeutralMode);
                printError(
                                kBackRightTurn.configAllSettings(kBackRightTurnConfig, GlobalConstants.kTimeoutMS));
                kBackRightTurn.setNeutralMode(kTurnNeutralMode);

                printError(
                                kFrontLeftDrive.configAllSettings(
                                                kFrontLeftDriveConfig, GlobalConstants.kTimeoutMS));
                kFrontLeftDrive.setNeutralMode(kDriveNeutralMode);
                printError(
                                kFrontRightDrive.configAllSettings(
                                                kFrontRightDriveConfig, GlobalConstants.kTimeoutMS));
                kFrontRightDrive.setNeutralMode(kDriveNeutralMode);
                printError(
                                kBackLeftDrive.configAllSettings(kBackLeftDriveConfig, GlobalConstants.kTimeoutMS));
                kBackLeftDrive.setNeutralMode(kDriveNeutralMode);
                printError(
                                kBackRightDrive.configAllSettings(
                                                kBackRightDriveConfig, GlobalConstants.kTimeoutMS));
                kBackRightDrive.setNeutralMode(kDriveNeutralMode);

                // set invert, same for all b/c all motors facing same direction
                kFrontLeftTurn.setInverted(kTurnMotorInverted);
                kFrontRightTurn.setInverted(kTurnMotorInverted);
                kBackLeftTurn.setInverted(kTurnMotorInverted);
                kBackRightTurn.setInverted(kTurnMotorInverted);

                kFrontLeftDrive.setInverted(kDriveMotorInverted);
                kFrontRightDrive.setInverted(kDriveMotorInverted);
                kBackLeftDrive.setInverted(kDriveMotorInverted);
                kBackRightDrive.setInverted(kDriveMotorInverted);
        }

        private SwerveDriveConstants() {
        }
}
