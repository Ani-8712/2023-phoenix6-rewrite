package team3647.frc2023.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.constants.FieldConstants;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.frc2023.util.SuperstructureState;

public class AutoCommands {
    private final SwerveDrive drive;
    private final SwerveDriveKinematics driveKinematics;
    private final Superstructure superstructure;

    public final AutonomousMode Test;

    public final AutonomousMode blueConeCubeBalanceFlatSideMode;
    public final AutonomousMode blueConeCubeBalanceBumpSideMode;
    public final AutonomousMode blueConeBalance;
    public final AutonomousMode blueJustScore;

    public final AutonomousMode redConeCubeBalanceFlatSideMode;
    public final AutonomousMode redConeCubeBalanceBumpSideMode;
    public final AutonomousMode redConeBalance;
    public final AutonomousMode redJustScore;

    public AutoCommands(
            SwerveDrive drive,
            SwerveDriveKinematics driveKinematics,
            Superstructure superstructure) {
        this.drive = drive;
        this.driveKinematics = driveKinematics;
        this.superstructure = superstructure;
        Test = new AutonomousMode(drive(), Trajectories.Blue.Test.kStart);
        // blue side modes
        blueConeCubeBalanceFlatSideMode =
                new AutonomousMode(
                        coneCubeBalanceFlatSide(Alliance.Blue),
                        Trajectories.Blue.ConeCubeBalanceFlatSide.kFirstPathInitial);
        blueConeCubeBalanceBumpSideMode =
                new AutonomousMode(
                        coneCubeBalanceBumpSide(Alliance.Blue),
                        Trajectories.Blue.ConeCubeBalanceBumpSide.kFirstPathInitial);
        blueConeBalance =
                new AutonomousMode(
                        justScoreBalance(
                                () -> SuperstructureState.coneThreeReversed, Alliance.Blue),
                        Trajectories.Blue.ConeBalance.kFirstPathInitial);
        blueJustScore =
                new AutonomousMode(
                        justScore(() -> SuperstructureState.coneThreeReversed),
                        flipForPP(getJustScore(FieldConstants.kBlueSix)));

        // red side modes
        redConeCubeBalanceFlatSideMode =
                new AutonomousMode(
                        coneCubeBalanceFlatSide(Alliance.Red),
                        flipForPP(Trajectories.Blue.ConeCubeBalanceFlatSide.kFirstPathInitial));
        redConeCubeBalanceBumpSideMode =
                new AutonomousMode(
                        coneCubeBalanceBumpSide(Alliance.Red),
                        flipForPP(Trajectories.Blue.ConeCubeBalanceBumpSide.kFirstPathInitial));

        redConeBalance =
                new AutonomousMode(
                        justScoreBalance(() -> SuperstructureState.coneThreeReversed, Alliance.Red),
                        flipForPP(getJustScore(FieldConstants.kRedFour)));
        redJustScore =
                new AutonomousMode(
                        justScore(() -> SuperstructureState.coneThreeReversed),
                        flipForPP(getJustScore(FieldConstants.kRedFour)));
    }

    public static Pose2d getJustScore(Pose2d pose) {
        return new Pose2d(
                pose.getTranslation(), pose.getRotation().rotateBy(FieldConstants.kOneEighty));
    }

    public Pose2d flipForPP(Pose2d pose) {
        return new Pose2d(
                new Translation2d(pose.getX(), FieldConstants.kFieldWidth - pose.getY()),
                new Rotation2d(pose.getRotation().getCos() * -1, pose.getRotation().getSin()));
    }

    private Command getSupestructureSequenceConeCubeFlat() {
        return Commands.sequence(
                justScore(() -> SuperstructureState.coneThreeReversed),
                Commands.waitSeconds(2),
                Commands.parallel(
                                superstructure.groundIntakeCube(),
                                superstructure.rollersCommands.intakeCube())
                        .withTimeout(2.5),
                superstructure.stow().withTimeout(2),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                superstructure.scoreAndStowCube(1));
    }

    private Command getSupestructureSequenceConeCubeBump() {
        return Commands.sequence(
                justScore(() -> SuperstructureState.coneThreeReversed),
                // longer weight so the intake doesn't deploy on the bump and kill itself
                Commands.waitSeconds(2.7),
                Commands.parallel(
                                superstructure.groundIntakeCube(),
                                superstructure.rollersCommands.intakeCube())
                        .withTimeout(2.5),
                superstructure.stow().withTimeout(2),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                superstructure.scoreAndStowCube(1));
    }

    public Command drive() {
        return followTrajectory(Trajectories.Blue.Test.kTrajectory);
    }

    public Command coneCubeBalanceBumpSide(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(2), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceBumpSide.kFirstTrajectory,
                                        color)),
                        // rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceBumpSide.kSecondTrajectory,
                                        color)),
                        Commands.waitSeconds(1),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceBumpSide.kGoToBalance,
                                        color)),
                        // lock wheels so no slip
                        superstructure.drivetrainCommands.robotRelativeDrive(
                                new Translation2d(), FieldConstants.kZero, 0.3));

        return Commands.parallel(drivetrainSequence, getSupestructureSequenceConeCubeBump());
    }

    public Command coneCubeBalanceFlatSide(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(1.75), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceFlatSide.kFirstTrajectory,
                                        color)),
                        // intake rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceFlatSide.kSecondTrajectory,
                                        color)),
                        Commands.waitSeconds(1.2),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceFlatSide.kGoToBalance,
                                        color)),
                        // lock wheels so no slip
                        superstructure.drivetrainCommands.robotRelativeDrive(
                                new Translation2d(), FieldConstants.kZero, 0.3));

        return Commands.parallel(drivetrainSequence, getSupestructureSequenceConeCubeFlat());
    }

    public Command justScore(Supplier<SuperstructureState> state) {
        return Commands.sequence(
                superstructure
                        .goToStateParallel(state.get())
                        .alongWith(superstructure.rollersCommands.outCone())
                        .withTimeout(1),
                Commands.waitSeconds(0.5),
                superstructure.scoreAndStowConeReversed());
    }

    public Command justScoreBalance(Supplier<SuperstructureState> state, Alliance alliance) {
        var drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(6),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeBalance.kFirstTrajectory, alliance)),
                        superstructure
                                .drivetrainCommands
                                .robotRelativeDrive(
                                        new Translation2d(), Rotation2d.fromDegrees(5), 0.3)
                                .withTimeout(0.2),
                        superstructure.drivetrainCommands.robotRelativeDrive(
                                new Translation2d(), FieldConstants.kZero, 0.3));
        return Commands.parallel(drivetrainSequence, justScore(state));
    }

    public AutonomousMode getJustScoreBlue(SuperstructureState state) {
        return new AutonomousMode(
                justScore(() -> state), new Pose2d(1.8, 3.26, FieldConstants.kZero));
    }

    public AutonomousMode getJustScoreRed(SuperstructureState state) {
        return new AutonomousMode(
                justScore(() -> state), flipForPP(new Pose2d(1.8, 3.26, FieldConstants.kZero)));
    }

    public Command followTrajectory(PathPlannerTrajectory trajectory) {
        return new PPSwerveControllerCommand(
                trajectory,
                drive::getOdoPose,
                driveKinematics,
                AutoConstants.kXController,
                AutoConstants.kYController,
                AutoConstants.kRotController,
                drive::setModuleStates,
                // false runs blue
                false,
                drive);
    }
}
