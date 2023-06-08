package team3647.frc2023.constants;

// import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class WristConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.WristIds.kMasterId);
    public static final boolean kMasterInvert = true;

    private static final double kGearBoxRatio = 1.0 / 70;
    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kNativePosToDegrees = kGearBoxRatio / GlobalConstants.kFalconTicksPerRotation * 360.0;

    public static final double kNativeVelToDPS = 10 * kNativePosToDegrees;

    private static final double masterKP = 0.5;
    private static final double masterKI = 0;
    private static final double masterKD = 0;

    public static final double nominalVoltage = 11.0;
    public static final double kStallCurrent = 20.0;
    public static final double kMaxCurrent = 15;

    public static final double kG = 0.0;

    public static final double kMaxVelocityTicks = (300.0) * 8;
    public static final double kMaxAccelerationTicks = (200.0) * 8;

    // public static final double kMinDegree = 5;
    // public static final double kMaxDegree = 135;

    public static final double kMinDegree = -90;
    public static final double kMaxDegree = 135;

    public static final double kInitialDegree = 140;
    public static final double kHoldPosition = 30;
    public static final double kDoubleStationDegrees = 106;
    public static final double kConeScoreAngle = 90;
    public static final double kCubeScoreAngle = 90;

    static {
        kMaster.getConfigurator().apply(new TalonFXConfiguration());

        kMasterConfig.Slot0.kP = masterKP;
        kMasterConfig.Slot0.kI = masterKI;
        kMasterConfig.Slot0.kD = masterKD;
        // kMasterConfig.Slot0.allowableClosedloopError = 100;

        kMasterConfig.MotionMagic.MotionMagicAcceleration = kMaxVelocityTicks;
        kMasterConfig.MotionMagic.MotionMagicCruiseVelocity = kMaxAccelerationTicks;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kMinDegree / kNativePosToDegrees;
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMaxDegree / kNativePosToDegrees;

        kMaster.setInverted(kMasterInvert);
        // kMaster.configGetStatorCurrentLimit(
        // new StatorCurrentLimitConfiguration(true, kStallCurrent, kMaxCurrent, 5));
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kMasterConfig.CurrentLimits.StatorCurrentLimit = kStallCurrent;

        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // kMaster.enableVoltageCompensation(true);
        kMaster.getConfigurator().apply(kMasterConfig);
    }

    private WristConstants() {
    }
}
