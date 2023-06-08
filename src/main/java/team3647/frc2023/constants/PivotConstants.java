package team3647.frc2023.constants;

// import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.InterpolatingTreeMap;

public class PivotConstants {
    // positive is swinging towards the front of the robot
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.PivotIds.kMasterId);
    public static final TalonFX kSlave = new TalonFX(GlobalConstants.PivotIds.kSlaveId);

    public static final boolean kMasterInvert = true;
    public static final boolean kSlaveInvert = kMasterInvert;

    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    // private static final double kGearBoxRatio = 1 / 91.022;
    private static final double kGearBoxRatio = 1 / 120.0;

    public static final double kNativePosToDegrees = kGearBoxRatio / GlobalConstants.kFalconTicksPerRotation * 360.0;

    public static final double kNativeVelToDPS = 10 * kNativePosToDegrees;

    public static final double kMaxVelocityTicks = (400) * 1.8;
    public static final double kMaxAccelerationTicks = (200.0) * 1.8;

    public static final double kMinDegree = -30.0;
    public static final double kMaxDegree = 210.0;

    // kG at max extension
    public static final double kG = 0.63;

    private static final double masterKP = 0.15;
    private static final double masterKI = 0;
    private static final double masterKD = 0;

    public static final double nominalVoltage = 11.0;
    public static final double kStallCurrent = 30.0;
    public static final double kMaxCurrent = 60.0;

    public static final double kMaxkG = 0.6;
    public static final double[][] kVoltageGravity = { { 0.1, 0.1 }, { Units.inchesToMeters(61), 0.55 } };

    public static final InterpolatingTreeMap<Double, Double> kLengthGravityVoltageMap = new InterpolatingTreeMap<>();
    public static final double kInitialAngle = 90.0;

    static {
        kMaster.getConfigurator().apply(new TalonFXConfiguration());
        kSlave.getConfigurator().apply(new TalonFXConfiguration());

        kMasterConfig.Slot0.kP = masterKP;
        kMasterConfig.Slot0.kI = masterKI;
        kMasterConfig.Slot0.kD = masterKD;
        // kMasterConfig.Slot0.allowableClosedloopError = 100;
        // kMasterConfig.voltageCompSaturation = nominalVoltage;
        kMasterConfig.MotionMagic.MotionMagicAcceleration = kMaxVelocityTicks;
        kMasterConfig.MotionMagic.MotionMagicCruiseVelocity = kMaxAccelerationTicks;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kMinDegree / kNativePosToDegrees;
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMaxDegree / kNativePosToDegrees;
        // kMasterConfig.peakOutputReverse = -0.8;

        // kSlave.follow(kMaster);
        kSlave.setControl(new Follower(kMaster.getDeviceID(), false));
        kMaster.setInverted(kMasterInvert);
        kSlave.setInverted(kSlaveInvert);
        // kMaster.configGetStatorCurrentLimit(
        // new StatorCurrentLimitConfiguration(true, kStallCurrent, kMaxCurrent, 3));

        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kMasterConfig.CurrentLimits.StatorCurrentLimit = kStallCurrent;

        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        var kSlaveMotorConfig = new MotorOutputConfigs();
        kSlaveMotorConfig.NeutralMode = NeutralModeValue.Brake;
        kSlave.getConfigurator().apply(kSlaveMotorConfig);
        // kMaster.enableVoltageCompensation(true);
        // kSlave.enableVoltageCompensation(true);

        for (double[] pair : kVoltageGravity) {
            kLengthGravityVoltageMap.put(pair[0], pair[1]);
        }
        kMaster.getConfigurator().apply(kMasterConfig, GlobalConstants.kTimeoutMS);
    }

    public static double getkGFromLength(double length) {
        double d = kLengthGravityVoltageMap.get(length);

        return MathUtil.clamp(d, 0.0, kMaxkG);
    }

    private PivotConstants() {
    }
}
