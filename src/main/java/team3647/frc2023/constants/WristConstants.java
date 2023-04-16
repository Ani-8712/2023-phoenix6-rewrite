package team3647.frc2023.constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public final class WristConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.WristIds.kMasterId);
    public static final InvertType kMasterInvert = InvertType.InvertMotorOutput;

    private static final double kGearBoxRatio = 1.0 / 70;
    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kNativePosToDegrees =
            kGearBoxRatio / GlobalConstants.kFalconTicksPerRotation * 360.0;

    public static final double kNativeVelToDPS = 10 * kNativePosToDegrees;

    private static final double masterKP = 0.5;
    private static final double masterKI = 0;
    private static final double masterKD = 0;

    public static final double nominalVoltage = 11.0;
    public static final double kStallCurrent = 20.0;
    public static final double kMaxCurrent = 15;

    public static final double kG = 0.0;

    public static final double kMaxVelocityTicks = (300.0 / kNativeVelToDPS) * 8;
    public static final double kMaxAccelerationTicks = (200.0 / kNativeVelToDPS) * 8;

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
        kMaster.configFactoryDefault();

        kMasterConfig.slot0.kP = masterKP;
        kMasterConfig.slot0.kI = masterKI;
        kMasterConfig.slot0.kD = masterKD;
        kMasterConfig.slot0.allowableClosedloopError = 100;
        kMasterConfig.voltageCompSaturation = nominalVoltage;
        kMasterConfig.motionAcceleration = kMaxVelocityTicks;
        kMasterConfig.motionCruiseVelocity = kMaxAccelerationTicks;
        kMasterConfig.reverseSoftLimitEnable = true;
        kMasterConfig.reverseSoftLimitThreshold = kMinDegree / kNativePosToDegrees;
        kMasterConfig.forwardSoftLimitEnable = true;
        kMasterConfig.forwardSoftLimitThreshold = kMaxDegree / kNativePosToDegrees;

        kMaster.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kMaster.setInverted(kMasterInvert);
        kMaster.configGetStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kStallCurrent, kMaxCurrent, 5));

        kMaster.setNeutralMode(NeutralMode.Brake);
        kMaster.enableVoltageCompensation(true);
    }

    private WristConstants() {}
}
