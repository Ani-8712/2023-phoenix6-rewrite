package team3647.frc2023.util;

import java.util.Map;
import team3647.frc2023.constants.ExtenderConstants;
import team3647.frc2023.constants.PivotConstants;
import team3647.frc2023.robot.PositionFinder.GamePiece;
import team3647.frc2023.robot.PositionFinder.Level;
import team3647.frc2023.subsystems.Superstructure.StationType;

public class SuperstructureState {
    /** degrees */
    public final double angle;

    public final double length;

    public final String name;

    // name parameter is just for debugging purposes

    private SuperstructureState(double angle, double length, String name) {
        this.angle = angle;
        this.length = length;
        this.name = name;
    }

    public static final SuperstructureState coneOne =
            new SuperstructureState(150, ExtenderConstants.kMinimumPositionTicks, "cone low");
    public static final SuperstructureState coneTwo =
            new SuperstructureState(139, ExtenderConstants.kLevelTwoExtendCone, "cone mid");
    public static final SuperstructureState coneThree =
            new SuperstructureState(141 - 3, ExtenderConstants.kLevelThreeExtendCone, "cone high");

    public static final SuperstructureState coneOneReversed =
            new SuperstructureState(
                    180 - 150, ExtenderConstants.kMinimumPositionTicks, "cone reversed low");
    public static final SuperstructureState coneTwoReversed =
            new SuperstructureState(
                    180 - 139, ExtenderConstants.kLevelTwoExtendCone, "cone reversed mid");
    public static final SuperstructureState coneThreeReversed =
            new SuperstructureState(
                    180 - (141 - 3), ExtenderConstants.kLevelThreeExtendCone, "cone reversed high");

    public static final SuperstructureState cubeOne =
            new SuperstructureState(125, ExtenderConstants.kMinimumPositionTicks, "cube low");
    public static final SuperstructureState cubeTwo =
            new SuperstructureState(149, ExtenderConstants.kLevelTwoExtendCube, "cube mid");
    public static final SuperstructureState cubeThree =
            new SuperstructureState(143, ExtenderConstants.kLevelThreeExtendCube, "cube high");

    public static final SuperstructureState cubeOneReversed =
            new SuperstructureState(
                    35, ExtenderConstants.kMinimumPositionTicks, "cube reversed low");
    public static final SuperstructureState cubeTwoReversed =
            new SuperstructureState(27, ExtenderConstants.kLevelTwoExtendCube, "cube reversed mid");
    public static final SuperstructureState cubeThreeReversed =
            new SuperstructureState(
                    180 - cubeThree.angle,
                    ExtenderConstants.kLevelThreeExtendCube,
                    "cube reversed high");

    public static final SuperstructureState noLevel =
            new SuperstructureState(
                    PivotConstants.kInitialAngle,
                    ExtenderConstants.kMinimumPositionTicks,
                    "no level");

    public static final SuperstructureState groundIntake =
            new SuperstructureState(190, ExtenderConstants.kMinimumPositionTicks, "ground intake");

    public static final SuperstructureState singleStation =
            new SuperstructureState(146, ExtenderConstants.kMinimumPositionTicks, "station");

    public static final SuperstructureState doubleStation =
            new SuperstructureState(139.5, ExtenderConstants.kGroundStation, "double station");
    public static final SuperstructureState stow =
            new SuperstructureState(0, ExtenderConstants.kMinimumPositionTicks, "double station");

    public static final Map<Level, Map<GamePiece, SuperstructureState>> kLevelPieceMap =
            Map.of(
                    Level.One,
                    Map.of(
                            GamePiece.Cone, coneOne,
                            GamePiece.Cube, cubeOne),
                    Level.Two,
                    Map.of(
                            GamePiece.Cone, coneTwo,
                            GamePiece.Cube, cubeTwo),
                    Level.Three,
                    Map.of(
                            GamePiece.Cone, coneThree,
                            GamePiece.Cube, cubeThree),
                    Level.Stay,
                    Map.of(
                            GamePiece.Cone, noLevel,
                            GamePiece.Cube, noLevel));
    public static final Map<StationType, SuperstructureState> kIntakePositionsMap =
            Map.of(
                    StationType.Double, doubleStation,
                    StationType.Single, singleStation,
                    StationType.Ground, groundIntake);
}