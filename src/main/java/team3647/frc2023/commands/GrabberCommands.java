package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2023.subsystems.Grabber;

public class GrabberCommands {

    public Command closeGrabber() {
        return Commands.run(() -> grabber.close(), this.grabber)
                .finallyDo(interrupted -> Commands.run(() -> {}, grabber));
    }

    public Command openGrabber() {
        return Commands.run(() -> grabber.open(), this.grabber)
                .finallyDo(interrupted -> Commands.run(() -> {}, grabber));
    }

    // public Command holdPosition() {
    //     return Commands.run(() -> grabber.holdCurrentPosition(), this.grabber)
    //             .finallyDo(interrupted -> Commands.run(() -> {}, grabber));
    // }

    private final Grabber grabber;

    public GrabberCommands(Grabber grabber) {
        this.grabber = grabber;
    }
}
