package team3647.frc2023.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import java.util.function.BooleanSupplier;
import team3647.frc2023.subsystems.SwerveDrive.PeriodicIO;
import team3647.lib.NetworkColorSensor;
import team3647.lib.NetworkColorSensor.GamePiece;
import team3647.lib.PeriodicSubsystem;

public class Grabber implements PeriodicSubsystem {
    private final Solenoid pistons;
    private final BooleanSupplier isOverrideCube;
    private final BooleanSupplier isOverrideCone;
    private final NetworkColorSensor colorSensor;
    private final PeriodicIO periodicIO = new PeriodicIO();

    public static class PeriodicIO {
        public GamePiece gamePiece = GamePiece.NONE;
        public boolean pistonOpen = false;
    }

    public Grabber(
            Solenoid pistons,
            NetworkColorSensor colorSensor,
            BooleanSupplier isOverrideCube,
            BooleanSupplier isOverrideCone) {
        this.pistons = pistons;
        this.colorSensor = colorSensor;
        this.isOverrideCone = isOverrideCone;
        this.isOverrideCube = isOverrideCube;
        close();
    }

    public void close() {
        periodicIO.pistonOpen = false;
    }

    public void open() {
        periodicIO.pistonOpen = true;
    }

    public GamePiece getGamepiece() {
        // return colorSensor.getGamepiece();
        return periodicIO.gamePiece;
    }

    public String getGamePieceStr() {
        return periodicIO.gamePiece.str;
    }

    @Override
    public void readPeriodicInputs() {
        if (isOverrideCone.getAsBoolean() == true) {
            periodicIO.gamePiece = GamePiece.CONE;
        } else if (isOverrideCube.getAsBoolean()) {
            periodicIO.gamePiece = GamePiece.CUBE;
        } else {
            // periodicIO.gamePiece = colorSensor.getGamepiece();
            periodicIO.gamePiece = GamePiece.NONE;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        pistons.set(periodicIO.pistonOpen);
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Grabber";
    }
}
