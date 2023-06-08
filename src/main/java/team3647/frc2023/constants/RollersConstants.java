package team3647.frc2023.constants;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DigitalInput;

public class RollersConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.RollersIds.kMasterId);
    public static final DigitalInput kCubeSensor = new DigitalInput(GlobalConstants.RollersIds.kSensorId);

    public static final InvertedValue kMasterInvert = InvertedValue.CounterClockwise_Positive;

    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kNominalVoltage = 11.0;
    public static final double kStallCurrent = 25.0;
    public static final double kMaxCurrent = 20.0;

    static {
        // kMaster.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);

        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kMasterConfig.CurrentLimits.StatorCurrentLimit = kStallCurrent;
        kMasterConfig.MotorOutput.Inverted = kMasterInvert;
        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // kMaster.enableVoltageCompensation(true);
        // kMaster.configVoltageCompSaturation(kNominalVoltage,
        // GlobalConstants.kTimeoutMS);
        // kMaster.configGetStatorCurrentLimit(
        // new StatorCurrentLimitConfiguration(true, kStallCurrent, kMaxCurrent, 5));

        kMaster.getConfigurator().apply(kMasterConfig);
    }

    private RollersConstants() {
    }
}
