package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import team3647.frc2023.subsystems.SwerveDrive.PeriodicIO;
import team3647.lib.TalonFXSubsystem;

public class Grabber extends TalonFXSubsystem {
    private final Solenoid pistons;
    private final DigitalInput gamePieceSensor;
    private final PeriodicIO periodicIO = new PeriodicIO();

    public static class PeriodicIO {
        public boolean hasGamePiece = false;
        public boolean pistonOpen = false;
    }

    public Grabber(
            TalonFX master,
            Solenoid pistons,
            DigitalInput gamePieceSensor,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        this.pistons = pistons;
        this.gamePieceSensor = gamePieceSensor;
        close();
    }

    public void close() {
        periodicIO.pistonOpen = false;
        super.setOpenloop(0.0);
    }

    public void open() {
        periodicIO.pistonOpen = true;
    }

    public void intakeCone() {
        periodicIO.pistonOpen = false;
        super.setOpenloop(0.4);
    }

    public void intakeCube() {
        periodicIO.pistonOpen = true;
        super.setOpenloop(0.4);
    }

    public void holdCurrentPosition() {
        super.setPosition(super.getPosition(), 0);
    }

    public boolean getHasGamePiece() {
        return periodicIO.hasGamePiece;
    }

    @Override
    public void readPeriodicInputs() {
        super.readPeriodicInputs();
        periodicIO.hasGamePiece = gamePieceSensor.get();
    }

    @Override
    public void writePeriodicOutputs() {
        super.writePeriodicOutputs();
        pistons.set(periodicIO.pistonOpen);
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Grabber";
    }
}
