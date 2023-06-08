package team3647.frc2023.constants;

// import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.playingwithfusion.TimeOfFlight;

public class CubeWristConstants {
        public static final TalonFX kMaster = new TalonFX(GlobalConstants.CubeWristIds.kMasterId);
        public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
        public static final TimeOfFlight timeOfFlight = new TimeOfFlight(GlobalConstants.CubeWristIds.timeOfFlightId);

        public static final boolean kMasterInvert = false;

        private static final double kGearBoxRatio = 12 / 76.0 * 18 / 80.0 * 18 / 48.0;

        public static final double kNativePosToDegrees = kGearBoxRatio / GlobalConstants.kFalconTicksPerRotation
                        * 360.0;

        public static final double kNativeVelToDPS = 10 * kNativePosToDegrees;

        public static final double kMaxVelocityTicks = (2000);
        public static final double kMaxAccelerationTicks = (2000.0);

        public static final double kInitialDegree = 0.0;
        public static final double kMinDegree = 0.0;
        public static final double kMaxDegree = 100.0;

        public static final double masterKP = 0.5;
        public static final double masterKI = 0.0;
        public static final double masterKD = 0.0;

        public static final double nominalVoltage = 11.0;
        public static final double kStallCurrent = 30.0;
        public static final double kMaxCurrent = 60.0;

        // kG at max extension
        public static final double kG = 0.00;

        static {
                kMaster.getConfigurator().apply(new TalonFXConfiguration());

                kMasterConfig.Slot0.kP = masterKP;
                kMasterConfig.Slot0.kI = masterKI;
                kMasterConfig.Slot0.kD = masterKD;
                // kMasterConfig.slot0.allowableClosedloopError = 100;
                // kMasterConfig.voltageCompSaturation = nominalVoltage;
                kMasterConfig.MotionMagic.MotionMagicAcceleration = kMaxVelocityTicks;
                kMasterConfig.MotionMagic.MotionMagicCruiseVelocity = kMaxAccelerationTicks;
                kMasterConfig.MotorOutput.PeakReverseDutyCycle = -0.5;
                kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
                kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kMinDegree / kNativePosToDegrees;
                kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
                kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMaxDegree / kNativePosToDegrees;

                kMasterConfig.CurrentLimits.StatorCurrentLimit = kStallCurrent;
                kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

                kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                kMaster.getConfigurator().apply(kMasterConfig, GlobalConstants.kTimeoutMS / 1000);
                kMaster.setInverted(kMasterInvert);
                // kMaster.configGetStatorCurrentLimit(
                // new StatorCurrentLimitConfiguration(true, kStallCurrent, kMaxCurrent, 3));

                // kMaster.setNeutralMode(NeutralMode.Brake);
                // kMaster.enableVoltageCompensation(true);
        }
}
