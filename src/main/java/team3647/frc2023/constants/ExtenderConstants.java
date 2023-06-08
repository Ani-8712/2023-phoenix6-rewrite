package team3647.frc2023.constants;

// import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class ExtenderConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.ExtenderIds.kMasterId);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    public static final boolean kMasterInvert = true;

    public static final double kRevTicksSoftLimit = 2000;

    private static final double kGearBoxRatio = 14.0 / 48.0 * 30.0 / 40.0 * 18.0 / 24.0;
    private static final double kDrumDiameterMeters = Units.inchesToMeters(1.2);

    public static final double kOutputRotationMeters = kDrumDiameterMeters * Math.PI * kGearBoxRatio;
    public static final double kNativePosToMeters = 1;
    // kOutputRotationMeters / GlobalConstants.kFalconTicksPerRotation;
    public static final double kNativeVelToMpS = 10 * kNativePosToMeters;

    public static final double kMaxVelocityMotorRotations = (30000.0 * 3.0) / GlobalConstants.kFalconTicksPerRotation;
    public static final double kMaxAccelerationMotorRotations = (30000.0 * 3.0)
            / GlobalConstants.kFalconTicksPerRotation;

    public static final double kMinimumPositionTicks = 2000;
    public static final double kMaximumPositionTicks = 56000.0;

    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public static final double nominalVoltage = 11.0;

    /** ticks */
    public static final double kLevelTwoExtendCone = 22601; // 32000 * 0.75;
    /** ticks */
    public static final double kLevelThreeExtendCone = 53760; // 78500 * 0.75;
    /** ticks */
    public static final double kLevelTwoExtendCube = 0; // 30000 * 0.75;
    /** ticks */
    public static final double kLevelThreeExtendCube = 31929; // 74000 * 0.75;

    public static final double kDoubleStation = 8600;

    static {
        kMaster.getConfigurator().apply(kMasterConfig);

        kMasterConfig.Slot0.kP = kP;
        kMasterConfig.Slot0.kI = kI;
        kMasterConfig.Slot0.kD = kD;
        kMasterConfig.MotionMagic.MotionMagicAcceleration = kMaxVelocityMotorRotations;
        kMasterConfig.MotionMagic.MotionMagicCruiseVelocity = kMaxAccelerationMotorRotations;
        // kMasterConfig.voltageCompSaturation = nominalVoltage;
        // kMasterConfig.slot0.allowableClosedloopError = 1000;

        kMasterConfig.MotorOutput.PeakReverseDutyCycle = -0.7;

        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kRevTicksSoftLimit;

        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kMaster.getConfigurator().apply(kMasterConfig, GlobalConstants.kTimeoutMS / 1000);
        kMaster.setInverted(kMasterInvert);
        // kMaster.enableVoltageCompensation(true);
    }
}
