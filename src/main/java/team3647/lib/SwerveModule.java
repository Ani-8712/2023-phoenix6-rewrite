package team3647.lib;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import team3647.frc2023.constants.SwerveDriveConstants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final SimpleMotorFeedforward ff;
    // abs since motor encoder resets
    public final CANcoder absEncoder;
    // offset reading of motor and abs pos after reset
    public final double absOffsetDeg;

    private final double driveVelocityConversion;
    private final double drivePositionConversion;
    private final double turnVelocityConversion;
    private final double turnPositionConversion;

    private DutyCycleOut dCycleOut;
    private VelocityVoltage velOut;
    private PositionDutyCycle posOut;

    private double lastAngle;

    public double percentOut = 0;

    public SwerveModule(
            TalonFX driveMotor,
            TalonFX turnMotor,
            SimpleMotorFeedforward ff,
            CANcoder absEncoder,
            double absOffsetDeg,
            double kDriveVelocityConversion,
            double kDrivePositionConversion,
            double kTurnVelocityConversion,
            double kTurnPositionConversion,
            double nominalVoltage) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.ff = ff;
        this.absEncoder = absEncoder;
        this.absOffsetDeg = absOffsetDeg;
        this.driveVelocityConversion = kDriveVelocityConversion;
        this.drivePositionConversion = kDrivePositionConversion;
        this.turnVelocityConversion = kTurnVelocityConversion;
        this.turnPositionConversion = kTurnPositionConversion;
        this.lastAngle = getState().angle.getDegrees();
        this.dCycleOut = new DutyCycleOut(0);
        this.velOut = new VelocityVoltage(0);
        this.posOut = new PositionDutyCycle(0);
        resetToAbsolute();
    }

    public double getDrivePos() {
        return driveMotor.getPosition().getValue() * drivePositionConversion;
    }

    public double getTurnAngle() {
        return turnMotor.getPosition().getValue() * turnPositionConversion;
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValue() * driveVelocityConversion;
    }

    public double getTurnVelocity() {
        return turnMotor.getVelocity().getValue() * turnVelocityConversion;
    }

    public Rotation2d getAbsEncoderPos() {
        return Rotation2d.fromDegrees(absEncoder.getAbsolutePosition().getValue() * 360);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurnAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePos(), Rotation2d.fromDegrees(getTurnAngle()));
    }

    public double getVoltage() {
        return driveMotor.getSupplyVoltage().getValue();
    }

    public void resetToAbsolute() {
        double absolutePosition = (getCANcoder() - absOffsetDeg) / turnPositionConversion;
        var error = turnMotor.setRotorPosition(absolutePosition, 0.255);

        // double absoluteAngle = (getAbsEncoderPos().getDegrees() - absOffsetDeg);
        // double adjustedAngle = CTREModuleState.optimizeAngle(absoluteAngle,
        // getTurnAngle());
        // turnMotor.setSelectedSensorPosition(adjustedAngle / turnPositionConversion);
    }

    public double getCANcoder() {
        return absEncoder.getAbsolutePosition().getValue() * 360;
    }

    public CANcoder getCANcoderObject() {
        return this.absEncoder;
    }

    public void resetDriveEncoders() {
        driveMotor.setRotorPosition(0);
    }

    public void setNeutralMode(NeutralModeValue turnNeutralMode, NeutralModeValue driveNeutralMode) {
        MotorOutputConfigs neutralModeConfig = new MotorOutputConfigs();
        turnMotor.getConfigurator().refresh(neutralModeConfig);
        neutralModeConfig.NeutralMode = turnNeutralMode;
        turnMotor.getConfigurator().apply(neutralModeConfig);

        driveMotor.getConfigurator().refresh(neutralModeConfig);
        neutralModeConfig.NeutralMode = driveNeutralMode;
        driveMotor.getConfigurator().apply(neutralModeConfig);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        desiredState = CTREModuleState.optimize(
                desiredState,
                getState().angle); // Custom optimize command, since default WPILib optimize
        // assumes continuous controller which CTRE is not

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond
                    / SwerveDriveConstants.kDrivePossibleMaxSpeedMPS;
            // driveMotor.set(ControlMode.PercentOutput, percentOutput);
            // this.percentOut = percentOutput;
            driveMotor.setControl(this.dCycleOut.withOutput(percentOutput));
            this.percentOut = percentOutput;
        } else {
            // MPS to falcon
            double velocity = desiredState.speedMetersPerSecond / driveVelocityConversion;
            // driveMotor.set(
            // ControlMode.Velocity,
            // velocity,
            // DemandType.ArbitraryFeedForward,
            // ff.calculate(desiredState.speedMetersPerSecond));
            driveMotor.setControl(this.velOut
                    .withVelocity(velocity)
                    .withFeedForward(ff.calculate(desiredState.speedMetersPerSecond)));
        }

        double angle = (Math
                .abs(desiredState.speedMetersPerSecond) <= (SwerveDriveConstants.kDrivePossibleMaxSpeedMPS * 0.01))
                        ? lastAngle
                        : desiredState.angle.getDegrees();
        // Prevents Jittering.
        // turnMotor.set(ControlMode.Position, angle / turnPositionConversion);
        turnMotor.setControl(posOut.withPosition(angle / turnPositionConversion).withFeedForward(0));
        lastAngle = angle;
    }

    public void stop() {
        // stop velocity, not set position to 0
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
