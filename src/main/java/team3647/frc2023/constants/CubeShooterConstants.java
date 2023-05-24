package team3647.frc2023.constants;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CubeShooterConstants {
    public static final TalonFX kTopRoller =
            new TalonFX(GlobalConstants.CubeShooterIds.kTopMasterId);
    public static final TalonFX kBottomRoller =
            new TalonFX(GlobalConstants.CubeShooterIds.kBottomMasterId);
    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    // public static final TalonFXInvertType kTopMotorInvert = TalonFXInvertType.Clockwise;
    public static final boolean kTopMotorInvert = true;
    // public static final TalonFXInvertType kBottomMotorInvert = TalonFXInvertType.Clockwise;
    public static final boolean kBottomMotorInvert = true;

    // kG at max extension
    public static final double kNominalVoltage = 11.0;
    public static final double kStallCurrent = 25.0;
    public static final double kMaxCurrent = 20.0;

    static {
        kTopRoller.getConfigurator().apply(kMasterConfig);
        // kTopRoller.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);

        kTopRoller.NeutralMode(NeutralModeValue.Brake);

        kTopRoller.enableVoltageCompensation(true);
        kTopRoller.configVoltageCompSaturation(kNominalVoltage, GlobalConstants.kTimeoutMS);
        kTopRoller.configGetStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kStallCurrent, kMaxCurrent, 5));

        kBottomRoller.setNeutralMode(NeutralModeValue.Brake);
        kBottomRoller.enableVoltageCompensation(true);
        kBottomRoller.configVoltageCompSaturation(kNominalVoltage, GlobalConstants.kTimeoutMS);
        kBottomRoller.configGetStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kStallCurrent, kMaxCurrent, 5));

        kTopRoller.setInverted(kTopMotorInvert);
        kBottomRoller.setInverted(kBottomMotorInvert);
    }
}
